// -----------------------------------------------------------------------------
// sim_main.cpp - Verilator testbench for mem_subsystem_top
//
// Replays a memory-access trace through 4 AXI client agents (GPU0, GPU1, CPU,
// DMA), with a scoreboard that checks every response:
//   * read data vs. shadow memory (byte-strobe aware)
//   * per-client in-order RID/BID, RLAST position, BRESP
//   * stray responses (e.g. after reset), deadlock watchdog, starvation bound
// and emits latency / throughput / fairness statistics as JSON.
// -----------------------------------------------------------------------------
#include <verilated.h>
#include <verilated_vcd_c.h>

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fstream>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "Vmem_subsystem_top.h"

namespace {

constexpr int      kClients        = 4;
constexpr int      kIdW            = 4;
constexpr uint32_t kRegionWords    = 0x4000 / 4;  // 16 KB per client
constexpr uint32_t kMemWords       = kClients * kRegionWords;
constexpr uint32_t k4KWords        = 1024;        // AXI: bursts never cross 4 KB
constexpr int      kMaxOutstanding = 8;
constexpr uint64_t kWatchdog       = 20000;
const char* const  kNames[kClients] = {"GPU0", "GPU1", "CPU", "DMA"};

struct Op {
  uint64_t cycle;
  int      client;
  bool     write;
  uint32_t addr;
  uint32_t len;  // AXI LEN (beats - 1)
};

struct Txn {
  bool                  write = false;
  uint32_t              word  = 0;  // global word index
  uint32_t              len   = 0;
  uint32_t              id    = 0;
  uint64_t              t_req = 0;
  std::vector<uint32_t> data;       // write payload / expected read data
  std::vector<uint8_t>  strb;
  std::vector<uint8_t>  check;      // read beat has a known expected value
  uint32_t              beat = 0;
};

struct Beat {
  uint32_t data;
  uint8_t  strb;
  bool     last;
};

struct Stats {
  uint64_t              reads = 0, writes = 0, beats = 0, beats_win = 0;
  uint64_t              offered_win = 0, dropped = 0;
  std::vector<uint32_t> lat;
};

struct Client {
  std::deque<Op>   todo;
  bool             presenting = false;
  Txn              cur;
  std::deque<Txn>  rd_q, wr_q;
  std::deque<Beat> wbeats;
  uint32_t         next_id = 0;
  uint32_t         seq     = 0;
  Stats            st;
};

struct Config {
  std::string trace, json, vcd, lat_csv;
  int         policy     = 0;
  std::string policy_str = "rr";
  uint32_t    weights[kClients] = {2, 2, 8, 2};
  bool        mem_stall  = false;
  double      bp         = 0.0;  // probability client deasserts RREADY/BREADY
  uint64_t    reset_at   = 0;
  uint64_t    max_cycles = 5'000'000;
  uint64_t    max_lat    = 0;
  int         watch      = -1;   // client the latency bound applies to (-1 = all)
  uint64_t    window     = 0;
  uint32_t    seed       = 1;
  bool        quiet      = false;
};

template <typename T>
uint32_t field(T v, int i, int w) {
  return static_cast<uint32_t>((static_cast<uint64_t>(v) >> (i * w)) & ((1ull << w) - 1));
}
template <typename T>
void set_field(T& v, int i, int w, uint32_t x) {
  const uint64_t mask = ((1ull << w) - 1) << (i * w);
  v = static_cast<T>((static_cast<uint64_t>(v) & ~mask) | ((static_cast<uint64_t>(x) << (i * w)) & mask));
}
inline bool bit(uint64_t v, int i) { return (v >> i) & 1u; }

uint32_t mix(uint32_t a, uint32_t b, uint32_t c) {
  uint32_t h = a * 0x9E3779B1u ^ (b + 0x7F4A7C15u) * 0x85EBCA77u ^ c * 0xC2B2AE3Du;
  h ^= h >> 15; h *= 0x2C1B3C6Du; h ^= h >> 12;
  return h;
}

uint32_t apply_strb(uint32_t old, uint32_t data, uint8_t strb) {
  uint32_t m = 0;
  for (int b = 0; b < 4; ++b) if (strb & (1u << b)) m |= 0xFFu << (8 * b);
  return (old & ~m) | (data & m);
}

std::vector<Op> load_trace(const std::string& path) {
  std::ifstream in(path);
  if (!in) { std::fprintf(stderr, "cannot open trace %s\n", path.c_str()); std::exit(2); }
  std::vector<Op> ops;
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream ss(line);
    Op op{}; std::string rw, addr;
    if (!(ss >> op.cycle >> op.client >> rw >> addr >> op.len)) continue;
    op.write = (rw == "W" || rw == "w");
    op.addr  = static_cast<uint32_t>(std::stoul(addr, nullptr, 0));
    if (op.client < 0 || op.client >= kClients || op.len > 255) continue;
    ops.push_back(op);
  }
  std::stable_sort(ops.begin(), ops.end(), [](const Op& a, const Op& b) { return a.cycle < b.cycle; });
  return ops;
}

class Bench {
 public:
  Bench(const Config& cfg, const std::vector<Op>& ops)
      : cfg_(cfg), top_(new Vmem_subsystem_top), rng_(cfg.seed),
        shadow_(kMemWords, 0), known_(kMemWords, 1), pend_rd_(kMemWords, 0), pend_wr_(kMemWords, 0) {
    for (const Op& op : ops) {
      clients_[op.client].todo.push_back(op);
      if (cfg_.window == 0 || op.cycle < cfg_.window) clients_[op.client].st.offered_win += op.len + 1;
    }
    if (!cfg_.vcd.empty()) {
      Verilated::traceEverOn(true);
      vcd_.reset(new VerilatedVcdC);
      top_->trace(vcd_.get(), 99);
      vcd_->open(cfg_.vcd.c_str());
    }
    top_->cfg_policy    = cfg_.policy;
    top_->cfg_mem_stall = cfg_.mem_stall;
    top_->cfg_weight    = 0;
    for (int c = 0; c < kClients; ++c) set_field(top_->cfg_weight, c, 4, cfg_.weights[c]);
  }

  ~Bench() { if (vcd_) vcd_->close(); top_->final(); }

  int run() {
    apply_reset(5);
    while (cyc_ < cfg_.max_cycles && errors_.size() < 20) {
      if (cfg_.reset_at && cyc_ == cfg_.reset_at) { mid_reset(); continue; }
      if (all_done()) break;
      step();
      if (busy() && cyc_ - last_progress_ > kWatchdog) {
        error("DEADLOCK: no handshake progress for %llu cycles (cycle %llu)",
              (unsigned long long)kWatchdog, (unsigned long long)cyc_);
        break;
      }
    }
    if (!all_done() && errors_.empty()) error("%s", "TIMEOUT: work still pending at max cycles");
    report();
    return errors_.empty() ? 0 : 1;
  }

 private:
  // ---------------------------------------------------------------- helpers
  template <typename... A>
  void error(const char* fmt, A... args) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), fmt, args...);
    errors_.emplace_back(buf);
  }

  bool busy() const {
    for (const Client& c : clients_) {
      if (c.presenting || !c.rd_q.empty() || !c.wr_q.empty() || !c.wbeats.empty()) return true;
      if (!c.todo.empty() && c.todo.front().cycle <= cyc_) return true;
    }
    return false;
  }
  bool all_done() const {
    for (const Client& c : clients_)
      if (c.presenting || !c.rd_q.empty() || !c.wr_q.empty() || !c.todo.empty()) return false;
    return true;
  }

  void eval_dump() {
    top_->eval();
    if (vcd_) vcd_->dump(time_++);
  }

  void tick() {
    top_->clk = 1; eval_dump();
    top_->clk = 0; eval_dump();
    ++cyc_;
  }

  void clear_inputs() {
    top_->s_arvalid = 0; top_->s_awvalid = 0; top_->s_wvalid = 0;
    top_->s_rready  = 0; top_->s_bready  = 0;
  }

  void apply_reset(int n) {
    top_->rst_n = 0;
    clear_inputs();
    for (int i = 0; i < n; ++i) tick();
    top_->rst_n = 1;
    last_progress_ = cyc_;
  }

  // Reset while traffic is in flight: everything outstanding is dropped and
  // memory touched by partially-completed writes becomes "unknown".
  void mid_reset() {
    if (!cfg_.quiet) std::printf("[%llu] asserting reset during activity\n", (unsigned long long)cyc_);
    for (Client& c : clients_) {
      auto undo = [&](const Txn& t, bool accepted) {
        for (uint32_t i = 0; i <= t.len; ++i) {
          if (t.write) { --pend_wr_[t.word + i]; if (accepted) known_[t.word + i] = 0; }
          else         { --pend_rd_[t.word + i]; }
        }
        ++c.st.dropped;
      };
      if (c.presenting) undo(c.cur, false);
      for (const Txn& t : c.rd_q) undo(t, true);
      for (const Txn& t : c.wr_q) undo(t, true);
      c.presenting = false;
      c.rd_q.clear(); c.wr_q.clear(); c.wbeats.clear();
    }
    apply_reset(4);
    resets_done_++;
  }

  bool hazard(const Txn& t) const {
    for (uint32_t i = 0; i <= t.len; ++i)
      if (t.write ? pend_rd_[t.word + i] : pend_wr_[t.word + i]) return true;
    return false;
  }

  Txn make_txn(int ci, const Op& op) {
    Client& c = clients_[ci];
    Txn t;
    t.write = op.write;
    t.len   = op.len;
    t.id    = c.next_id++ & ((1u << kIdW) - 1);
    t.t_req = op.cycle;
    uint32_t off = (op.addr / 4) % kRegionWords;
    if (off + op.len + 1 > kRegionWords) off = kRegionWords - (op.len + 1);
    if (off / k4KWords != (off + op.len) / k4KWords) off = (off / k4KWords + 1) * k4KWords - (op.len + 1);
    t.word = ci * kRegionWords + off;
    std::uniform_int_distribution<int> pct(0, 99);
    for (uint32_t i = 0; i <= op.len; ++i) {
      if (t.write) {
        t.data.push_back(mix(ci, t.word + i, c.seq++));
        t.strb.push_back((ci == 2 && op.len == 0 && pct(rng_) < 30) ? (uint8_t)(1 + pct(rng_) % 15) : 0xF);
      } else {
        t.data.push_back(shadow_[t.word + i]);
        t.check.push_back(known_[t.word + i]);
      }
    }
    return t;
  }

  // ---------------------------------------------------------------- per cycle
  void drive() {
    clear_inputs();
    std::uniform_real_distribution<double> u(0.0, 1.0);
    for (int ci = 0; ci < kClients; ++ci) {
      Client& c = clients_[ci];
      const size_t outstanding = c.rd_q.size() + c.wr_q.size();
      if (!c.presenting && !c.todo.empty() && c.todo.front().cycle <= cyc_ && outstanding < kMaxOutstanding) {
        Txn t = make_txn(ci, c.todo.front());
        if (!hazard(t)) {
          c.todo.pop_front();
          for (uint32_t i = 0; i <= t.len; ++i) (t.write ? pend_wr_ : pend_rd_)[t.word + i]++;
          c.cur = std::move(t);
          c.presenting = true;
        } else {
          c.next_id--;
        }
      }
      if (c.presenting) {
        const uint32_t addr = c.cur.word * 4;
        if (c.cur.write) {
          top_->s_awvalid |= 1u << ci;
          top_->s_awaddr[ci] = addr;
          set_field(top_->s_awlen, ci, 8, c.cur.len);
          set_field(top_->s_awid, ci, kIdW, c.cur.id);
        } else {
          top_->s_arvalid |= 1u << ci;
          top_->s_araddr[ci] = addr;
          set_field(top_->s_arlen, ci, 8, c.cur.len);
          set_field(top_->s_arid, ci, kIdW, c.cur.id);
        }
      }
      if (!c.wbeats.empty()) {
        const Beat& b = c.wbeats.front();
        top_->s_wvalid |= 1u << ci;
        top_->s_wdata[ci] = b.data;
        set_field(top_->s_wstrb, ci, 4, b.strb);
        if (b.last) top_->s_wlast |= 1u << ci; else top_->s_wlast &= ~(1u << ci);
      }
      if (u(rng_) >= cfg_.bp) top_->s_rready |= 1u << ci;
      if (u(rng_) >= cfg_.bp) top_->s_bready |= 1u << ci;
    }
  }

  void complete(Client& c, const Txn& t, int ci) {
    const uint64_t lat = cyc_ - t.t_req;
    c.st.lat.push_back(static_cast<uint32_t>(lat));
    if (cfg_.max_lat && lat > cfg_.max_lat && !starved_ && (cfg_.watch < 0 || cfg_.watch == ci)) {
      starved_ = true;
      error("STARVATION: %s waited %llu cycles (limit %llu)", kNames[ci],
            (unsigned long long)lat, (unsigned long long)cfg_.max_lat);
    }
  }

  void sample() {
    const uint32_t ar = top_->s_arvalid & top_->s_arready;
    const uint32_t aw = top_->s_awvalid & top_->s_awready;
    const uint32_t w  = top_->s_wvalid  & top_->s_wready;
    const uint32_t r  = top_->s_rvalid  & top_->s_rready;
    const uint32_t b  = top_->s_bvalid  & top_->s_bready;
    if (ar | aw | w | r | b) last_progress_ = cyc_;
    const bool in_win = cfg_.window == 0 || cyc_ < cfg_.window;

    for (int ci = 0; ci < kClients; ++ci) {
      Client& c = clients_[ci];
      if (bit(ar, ci)) {
        c.rd_q.push_back(std::move(c.cur));
        c.presenting = false;
      }
      if (bit(aw, ci)) {
        Txn& t = c.cur;
        for (uint32_t i = 0; i <= t.len; ++i) c.wbeats.push_back({t.data[i], t.strb[i], i == t.len});
        c.wr_q.push_back(std::move(c.cur));
        c.presenting = false;
      }
      if (bit(w, ci)) {
        if (c.wbeats.empty()) error("W accepted from %s with no data pending", kNames[ci]);
        else c.wbeats.pop_front();
      }
      if (bit(r, ci)) {
        if (c.rd_q.empty()) {
          error("STRAY R beat delivered to %s at cycle %llu (no read outstanding)", kNames[ci], (unsigned long long)cyc_);
          continue;
        }
        Txn& t = c.rd_q.front();
        const uint32_t rid  = field(top_->s_rid, ci, kIdW);
        const bool     last = bit(top_->s_rlast, ci);
        const uint32_t data = top_->s_rdata[ci];
        if (rid != t.id)
          error("ORDER: %s got RID %u, expected %u (cycle %llu)", kNames[ci], rid, t.id, (unsigned long long)cyc_);
        if (last != (t.beat == t.len))
          error("RLAST: %s beat %u/%u rlast=%d (cycle %llu)", kNames[ci], t.beat, t.len, last, (unsigned long long)cyc_);
        if (t.beat < t.check.size() && t.check[t.beat] && data != t.data[t.beat])
          error("DATA: %s addr 0x%05x got 0x%08x expected 0x%08x (cycle %llu)", kNames[ci],
                (t.word + t.beat) * 4, data, t.data[t.beat], (unsigned long long)cyc_);
        ++t.beat; ++c.st.beats; if (in_win) ++c.st.beats_win;
        if (t.beat > t.len) {
          for (uint32_t i = 0; i <= t.len; ++i) --pend_rd_[t.word + i];
          ++c.st.reads;
          complete(c, t, ci);
          c.rd_q.pop_front();
        }
      }
      if (bit(b, ci)) {
        if (c.wr_q.empty()) {
          error("STRAY B response delivered to %s at cycle %llu", kNames[ci], (unsigned long long)cyc_);
          continue;
        }
        Txn& t = c.wr_q.front();
        const uint32_t bid = field(top_->s_bid, ci, kIdW);
        if (bid != t.id) error("ORDER: %s got BID %u, expected %u", kNames[ci], bid, t.id);
        if (field(top_->s_bresp, ci, 2) != 0) error("BRESP error for %s", kNames[ci]);
        for (uint32_t i = 0; i <= t.len; ++i) {
          const uint32_t wd = t.word + i;
          if (known_[wd]) shadow_[wd] = apply_strb(shadow_[wd], t.data[i], t.strb[i]);
          else if (t.strb[i] == 0xF) { shadow_[wd] = t.data[i]; known_[wd] = 1; }
          --pend_wr_[wd];
        }
        c.st.beats += t.len + 1; if (in_win) c.st.beats_win += t.len + 1;
        ++c.st.writes;
        complete(c, t, ci);
        c.wr_q.pop_front();
      }
    }
  }

  void step() {
    drive();
    eval_dump();
    sample();
    tick();
  }

  // ---------------------------------------------------------------- report
  static uint32_t pct(std::vector<uint32_t> v, double p) {
    if (v.empty()) return 0;
    std::sort(v.begin(), v.end());
    size_t idx = static_cast<size_t>(p * (v.size() - 1) + 0.5);
    return v[std::min(idx, v.size() - 1)];
  }

  void report() {
    uint64_t total = 0, total_win = 0;
    for (const Client& c : clients_) { total += c.st.beats; total_win += c.st.beats_win; }
    const uint64_t win = cfg_.window ? std::min<uint64_t>(cfg_.window, cyc_) : cyc_;

    if (!cfg_.quiet) {
      std::printf("\n policy=%s  cycles=%llu  throughput=%.3f beats/cycle  stall=%d  bp=%.2f\n",
                  cfg_.policy_str.c_str(), (unsigned long long)cyc_,
                  cyc_ ? double(total) / cyc_ : 0.0, cfg_.mem_stall, cfg_.bp);
      std::printf(" %-5s %7s %7s %8s %8s %8s %8s\n", "client", "reads", "writes", "beats", "p50", "p99", "max");
      for (int ci = 0; ci < kClients; ++ci) {
        const Stats& s = clients_[ci].st;
        std::printf(" %-5s %7llu %7llu %8llu %8u %8u %8u\n", kNames[ci], (unsigned long long)s.reads,
                    (unsigned long long)s.writes, (unsigned long long)s.beats, pct(s.lat, 0.5),
                    pct(s.lat, 0.99), pct(s.lat, 1.0));
      }
    }

    for (size_t i = 0; i < errors_.size() && i < 10; ++i) std::printf(" ERROR: %s\n", errors_[i].c_str());
    std::printf("%s\n", errors_.empty() ? "PASS" : "FAIL");

    if (!cfg_.json.empty()) {
      std::ofstream js(cfg_.json);
      js << "{\n  \"policy\": \"" << cfg_.policy_str << "\",\n  \"cycles\": " << cyc_
         << ",\n  \"window\": " << win << ",\n  \"total_beats\": " << total
         << ",\n  \"throughput\": " << (cyc_ ? double(total) / cyc_ : 0.0)
         << ",\n  \"throughput_window\": " << (win ? double(total_win) / win : 0.0)
         << ",\n  \"pass\": " << (errors_.empty() ? "true" : "false") << ",\n  \"errors\": [";
      for (size_t i = 0; i < errors_.size(); ++i) js << (i ? ", " : "") << "\"" << errors_[i] << "\"";
      js << "],\n  \"clients\": [\n";
      for (int ci = 0; ci < kClients; ++ci) {
        const Stats& s = clients_[ci].st;
        double mean = 0;
        for (uint32_t l : s.lat) mean += l;
        if (!s.lat.empty()) mean /= s.lat.size();
        js << "    {\"name\": \"" << kNames[ci] << "\", \"reads\": " << s.reads << ", \"writes\": " << s.writes
           << ", \"beats\": " << s.beats << ", \"beats_window\": " << s.beats_win
           << ", \"offered_window\": " << s.offered_win << ", \"dropped\": " << s.dropped
           << ", \"lat_mean\": " << mean << ", \"lat_p50\": " << pct(s.lat, 0.5)
           << ", \"lat_p95\": " << pct(s.lat, 0.95) << ", \"lat_p99\": " << pct(s.lat, 0.99)
           << ", \"lat_max\": " << pct(s.lat, 1.0) << "}" << (ci + 1 < kClients ? "," : "") << "\n";
      }
      js << "  ]\n}\n";
    }
    if (!cfg_.lat_csv.empty()) {
      std::ofstream cs(cfg_.lat_csv);
      cs << "client,latency\n";
      for (int ci = 0; ci < kClients; ++ci)
        for (uint32_t l : clients_[ci].st.lat) cs << kNames[ci] << "," << l << "\n";
    }
  }

  Config                               cfg_;
  std::unique_ptr<Vmem_subsystem_top>  top_;
  std::unique_ptr<VerilatedVcdC>       vcd_;
  std::mt19937                         rng_;
  Client                               clients_[kClients];
  std::vector<uint32_t>                shadow_;
  std::vector<uint8_t>                 known_;
  std::vector<uint16_t>                pend_rd_, pend_wr_;
  std::vector<std::string>             errors_;
  uint64_t                             cyc_ = 0, time_ = 0, last_progress_ = 0;
  int                                  resets_done_ = 0;
  bool                                 starved_ = false;
};

void usage() {
  std::puts(
      "usage: Vmem_subsystem_top --trace FILE [options]\n"
      "  --policy rr|wrr|prio   arbitration policy (default rr)\n"
      "  --weights a,b,c,d      WRR weights for GPU0,GPU1,CPU,DMA (default 2,2,8,2)\n"
      "  --stall                enable pseudo-random memory stalls\n"
      "  --bp P                 client RREADY/BREADY backpressure probability\n"
      "  --reset-at N           assert reset at cycle N while traffic is in flight\n"
      "  --max-lat N            fail if any request latency exceeds N (starvation)\n"
      "  --watch C              apply --max-lat only to client C\n"
      "  --window N             measurement window for throughput/fairness\n"
      "  --max-cycles N         simulation limit\n"
      "  --json FILE  --lat-csv FILE  --vcd FILE  --seed N  --quiet");
}

}  // namespace

int main(int argc, char** argv) {
  Verilated::commandArgs(argc, argv);
  Config cfg;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) { usage(); std::exit(2); }
      return argv[++i];
    };
    if (a == "--trace") cfg.trace = next();
    else if (a == "--policy") {
      cfg.policy_str = next();
      cfg.policy = cfg.policy_str == "wrr" ? 1 : cfg.policy_str == "prio" ? 2 : 0;
    } else if (a == "--weights") {
      std::string w = next();
      std::sscanf(w.c_str(), "%u,%u,%u,%u", &cfg.weights[0], &cfg.weights[1], &cfg.weights[2], &cfg.weights[3]);
    }
    else if (a == "--stall") cfg.mem_stall = true;
    else if (a == "--bp") cfg.bp = std::stod(next());
    else if (a == "--reset-at") cfg.reset_at = std::stoull(next());
    else if (a == "--max-lat") cfg.max_lat = std::stoull(next());
    else if (a == "--watch") cfg.watch = std::stoi(next());
    else if (a == "--window") cfg.window = std::stoull(next());
    else if (a == "--max-cycles") cfg.max_cycles = std::stoull(next());
    else if (a == "--json") cfg.json = next();
    else if (a == "--lat-csv") cfg.lat_csv = next();
    else if (a == "--vcd") cfg.vcd = next();
    else if (a == "--seed") cfg.seed = static_cast<uint32_t>(std::stoul(next()));
    else if (a == "--quiet") cfg.quiet = true;
    else if (a.rfind("+", 0) == 0) continue;
    else { usage(); return 2; }
  }
  if (cfg.trace.empty()) { usage(); return 2; }

  Bench bench(cfg, load_trace(cfg.trace));
  return bench.run();
}
