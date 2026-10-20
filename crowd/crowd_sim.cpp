// -----------------------------------------------------------------------------
// crowd_sim.cpp - crowd simulation instrumented as a memory-trace source
//
// A compact agent-based crowd engine (spatial-hash neighbour search,
// separation + goal steering) whose kernels are mapped onto SoC clients:
//
//   GPU0  integrate kernel   streams pos/vel arrays (SoA)          -> long bursts
//   GPU1  neighbour kernel   gathers neighbour positions           -> locality-dependent
//   CPU   grid rebuild       scattered cell-counter RMW            -> single beats
//   DMA   frame export       bulk copy of positions every K steps  -> 32-beat bursts
//
// Per-lane word addresses are coalesced per 32-lane warp into AXI INCR bursts
// (max 16 beats), the same way a GPU memory coalescer merges a warp's
// requests. Locality therefore shows up directly as burst length.
//
// Scenarios:
//   dense    agents in a tight cluster, re-sorted by cell each step
//   sparse   agents scattered, never sorted -> neighbour gathers are random
//   evac     dense crowd rushing an exit, frame export every step (bursty)
//
// To drive the interconnect from a different engine, emit the same trace
// format through TraceWriter: "<cycle> <client> <R|W> <byte_addr> <axi_len>".
// -----------------------------------------------------------------------------
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <numeric>
#include <random>
#include <string>
#include <vector>

namespace {

constexpr int kWarp      = 32;
constexpr int kMaxBurst  = 16;
constexpr int kDmaBurst  = 32;

struct Burst {
  uint64_t cycle;
  int      client;
  bool     write;
  uint32_t word;
  uint32_t beats;
};

class TraceWriter {
 public:
  void add(int client, bool write, uint32_t word, uint32_t beats) {
    pending_[client].push_back({0, client, write, word, beats});
  }
  // spread this step's bursts for each client evenly over the frame budget
  void end_step(uint64_t step_start, uint64_t budget) {
    for (int c = 0; c < 4; ++c) {
      auto& v = pending_[c];
      for (size_t i = 0; i < v.size(); ++i) {
        v[i].cycle = step_start + (v.size() > 1 ? i * (budget * 9 / 10) / v.size() : 0);
        out_.push_back(v[i]);
      }
      v.clear();
    }
  }
  void save(const char* path, const std::string& header) {
    std::stable_sort(out_.begin(), out_.end(), [](const Burst& a, const Burst& b) { return a.cycle < b.cycle; });
    FILE* f = std::fopen(path, "w");
    if (!f) { std::perror(path); std::exit(1); }
    std::fprintf(f, "# crowd_sim %s\n# cycle client R/W addr len\n", header.c_str());
    for (const Burst& b : out_)
      std::fprintf(f, "%llu %d %c 0x%05x %u\n", (unsigned long long)b.cycle, b.client, b.write ? 'W' : 'R',
                   b.word * 4, b.beats - 1);
    std::fclose(f);
  }
  const std::vector<Burst>& bursts() const { return out_; }

 private:
  std::vector<Burst> pending_[4];
  std::vector<Burst> out_;
};

// Merge a warp's word addresses into maximal contiguous bursts.
void coalesce(TraceWriter& tw, int client, bool write, std::vector<uint32_t>& words) {
  if (words.empty()) return;
  std::sort(words.begin(), words.end());
  words.erase(std::unique(words.begin(), words.end()), words.end());
  uint32_t start = words[0], len = 1;
  for (size_t i = 1; i < words.size(); ++i) {
    if (words[i] == start + len && len < kMaxBurst) { ++len; continue; }
    tw.add(client, write, start, len);
    start = words[i]; len = 1;
  }
  tw.add(client, write, start, len);
  words.clear();
}

struct Agent { float x, y, vx, vy; };

class Crowd {
 public:
  Crowd(const std::string& scenario, int n, uint32_t seed)
      : scen_(scenario), n_(n), rng_(seed), agents_(n), order_(n) {
    std::normal_distribution<float>       blob(kWorld / 2, kWorld / 10);
    std::uniform_real_distribution<float> uni(0, kWorld);
    for (auto& a : agents_) {
      if (scen_ == "sparse") { a.x = uni(rng_); a.y = uni(rng_); }
      else { a.x = clamp(blob(rng_)); a.y = clamp(blob(rng_)); }
      a.vx = a.vy = 0;
    }
    std::iota(order_.begin(), order_.end(), 0);
    if (scen_ == "sparse") std::shuffle(order_.begin(), order_.end(), rng_);
  }

  void step(TraceWriter& tw, int s) {
    rebuild_grid(tw);
    neighbours(tw);
    integrate(tw);
    if (scen_ == "evac" || s % 4 == 3) export_frame(tw);
  }

 private:
  static constexpr float kWorld = 64.f;
  static constexpr int   kCells = 16;  // 16x16 grid
  static constexpr float kCellSz = kWorld / kCells;

  // memory map (word addresses inside each client's 16 KB region)
  static uint32_t pos_word(int i) { return 2 * i; }                // x,y SoA-interleaved
  static uint32_t vel_word(int i) { return 2048 + 2 * i; }
  static uint32_t cell_word(int c) { return 3584 + c; }

  static float clamp(float v) { return std::min(std::max(v, 0.f), kWorld - 0.01f); }
  int cell_of(const Agent& a) const { return int(a.y / kCellSz) * kCells + int(a.x / kCellSz); }

  // CPU: count agents per cell (scattered read-modify-write), optional sort
  void rebuild_grid(TraceWriter& tw) {
    cell_start_.assign(kCells * kCells + 1, 0);
    for (int k = 0; k < n_; ++k) {
      const int c = cell_of(agents_[order_[k]]);
      cell_start_[c + 1]++;
      if (k % 4 == 0) {  // sampled to keep traces compact
        tw.add(2, false, cell_word(c), 1);
        tw.add(2, true, cell_word(c), 1);
      }
    }
    for (int c = 0; c < kCells * kCells; ++c) cell_start_[c + 1] += cell_start_[c];
    if (scen_ != "sparse") {   // counting sort keeps neighbours contiguous in memory
      std::vector<int> fill(cell_start_.begin(), cell_start_.end() - 1), sorted(n_);
      std::vector<Agent> re(n_);
      for (int k = 0; k < n_; ++k) { int i = order_[k]; sorted[fill[cell_of(agents_[i])]++] = i; }
      for (int k = 0; k < n_; ++k) re[k] = agents_[sorted[k]];
      agents_.swap(re);
      std::iota(order_.begin(), order_.end(), 0);
    } else {
      bucket_.assign(kCells * kCells, {});
      for (int i = 0; i < n_; ++i) bucket_[cell_of(agents_[i])].push_back(i);
    }
  }

  std::vector<int> members(int c) const {
    if (scen_ != "sparse") {
      std::vector<int> v(cell_start_[c + 1] - cell_start_[c]);
      std::iota(v.begin(), v.end(), cell_start_[c]);
      return v;
    }
    return bucket_[c];
  }

  // GPU1: separation force from up to 8 neighbours in the 3x3 cell block
  void neighbours(TraceWriter& tw) {
    force_.assign(n_, {0, 0, 0, 0});
    std::vector<uint32_t> warp;
    for (int k = 0; k < n_; ++k) {
      const int i = order_[k];
      const Agent& a = agents_[i];
      const int cx = int(a.x / kCellSz), cy = int(a.y / kCellSz);
      int seen = 0;
      for (int dy = -1; dy <= 1 && seen < 8; ++dy)
        for (int dx = -1; dx <= 1 && seen < 8; ++dx) {
          const int x = cx + dx, y = cy + dy;
          if (x < 0 || y < 0 || x >= kCells || y >= kCells) continue;
          for (int j : members(y * kCells + x)) {
            if (j == i || seen >= 8) continue;
            const float ddx = a.x - agents_[j].x, ddy = a.y - agents_[j].y;
            const float d2 = ddx * ddx + ddy * ddy + 1e-3f;
            if (d2 < 4.f) { force_[i].x += ddx / d2; force_[i].y += ddy / d2; }
            warp.push_back(pos_word(j)); warp.push_back(pos_word(j) + 1);
            ++seen;
          }
        }
      if ((k + 1) % kWarp == 0) coalesce(tw, 1, false, warp);
    }
    coalesce(tw, 1, false, warp);
  }

  // GPU0: velocity/position integration, streaming SoA access
  void integrate(TraceWriter& tw) {
    const float gx = (scen_ == "evac") ? kWorld - 1 : kWorld / 2, gy = kWorld / 2;
    std::vector<uint32_t> rd, wr;
    for (int k = 0; k < n_; ++k) {
      const int i = order_[k];
      Agent& a = agents_[i];
      float tx = gx - a.x, ty = gy - a.y;
      const float tl = std::sqrt(tx * tx + ty * ty) + 1e-3f;
      a.vx = 0.8f * a.vx + 0.2f * (tx / tl) + 0.05f * force_[i].x;
      a.vy = 0.8f * a.vy + 0.2f * (ty / tl) + 0.05f * force_[i].y;
      a.x = clamp(a.x + 0.5f * a.vx);
      a.y = clamp(a.y + 0.5f * a.vy);
      for (uint32_t w : {pos_word(i), pos_word(i) + 1, vel_word(i), vel_word(i) + 1}) { rd.push_back(w); wr.push_back(w); }
      if ((k + 1) % kWarp == 0) { coalesce(tw, 0, false, rd); coalesce(tw, 0, true, wr); }
    }
    coalesce(tw, 0, false, rd);
    coalesce(tw, 0, true, wr);
  }

  // DMA: copy the position buffer to a frame buffer for rendering / logging
  void export_frame(TraceWriter& tw) {
    for (uint32_t w = 0; w < uint32_t(2 * n_); w += kDmaBurst) {
      tw.add(3, false, w, kDmaBurst);
      tw.add(3, true, 2048 + w, kDmaBurst);
    }
  }

  std::string        scen_;
  int                n_;
  std::mt19937       rng_;
  std::vector<Agent> agents_, force_;
  std::vector<int>   order_, cell_start_;
  std::vector<std::vector<int>> bucket_;
};

}  // namespace

int main(int argc, char** argv) {
  std::string scenario = "dense", out = "crowd.trace";
  int agents = 512, steps = 8;
  uint64_t budget = 12000;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto next = [&]() { return std::string(i + 1 < argc ? argv[++i] : ""); };
    if (a == "--scenario") scenario = next();
    else if (a == "--agents") agents = std::stoi(next());
    else if (a == "--steps") steps = std::stoi(next());
    else if (a == "--budget") budget = std::stoull(next());
    else if (a == "-o") out = next();
    else { std::fprintf(stderr, "usage: crowd_sim --scenario dense|sparse|evac [--agents N] [--steps N] [--budget CYC] -o FILE\n"); return 2; }
  }
  agents = std::min(agents, 1024);

  Crowd crowd(scenario, agents, 42);
  TraceWriter tw;
  for (int s = 0; s < steps; ++s) {
    crowd.step(tw, s);
    tw.end_step(uint64_t(s) * budget, budget);
  }

  uint64_t beats[4] = {}, bursts[4] = {};
  for (const Burst& b : tw.bursts()) { beats[b.client] += b.beats; bursts[b.client]++; }
  char hdr[256];
  std::snprintf(hdr, sizeof(hdr),
                "scenario=%s agents=%d steps=%d budget=%llu avg_burst=[%.2f,%.2f,%.2f,%.2f]",
                scenario.c_str(), agents, steps, (unsigned long long)budget,
                bursts[0] ? double(beats[0]) / bursts[0] : 0, bursts[1] ? double(beats[1]) / bursts[1] : 0,
                bursts[2] ? double(beats[2]) / bursts[2] : 0, bursts[3] ? double(beats[3]) / bursts[3] : 0);
  tw.save(out.c_str(), hdr);
  std::printf("%s: %zu bursts  %s\n", out.c_str(), tw.bursts().size(), hdr);
  return 0;
}
