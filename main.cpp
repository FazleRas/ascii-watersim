// watersim -- interactive ASCII water simulator
//
//   click     splash (drag to carve streams)
//   L         toggle land mode: click/drag builds 3x3 land, right-click erases
//   WASD      drive the active boat
//   b / TAB   spawn boat / switch boats
//   SPACE     toggle rain
//   q         quit
//
// Build:  g++ -std=c++17 -O2 main.cpp -o watersim
// Run:    ./watersim   (or --demo for a non-interactive smoke test)

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

// RAII: raw mode + mouse reporting on construction, restored on any exit path.
class Terminal {
public:
    Terminal() {
        ok_ = (tcgetattr(STDIN_FILENO, &orig_) == 0);
        if (!ok_) return;

        termios raw = orig_;
        raw.c_lflag &= ~(ECHO | ICANON | ISIG);
        raw.c_cc[VMIN] = 0;   // non-blocking reads
        raw.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);

        // alt screen, hide cursor, SGR mouse click+drag reporting
        std::cout << "\x1b[?1049h\x1b[?25l\x1b[?1002h\x1b[?1006h" << std::flush;
    }

    ~Terminal() {
        if (!ok_) return;
        std::cout << "\x1b[?1006l\x1b[?1002l\x1b[?25h\x1b[?1049l" << std::flush;
        tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_);
    }

    bool ok() const { return ok_; }

private:
    termios orig_{};
    bool ok_ = false;
};

// Height-field wave sim over two buffers. Each cell is driven by its four
// neighbours minus its previous value, which preserves outward momentum;
// damping bleeds off energy. Stored flat, indexed [row * width + col].
class Water {
public:
    Water(int width, int height)
        : w_(width), h_(height),
          cur_(width * height, 0.0),
          prev_(width * height, 0.0),
          land_(width * height, 0) {}

    int width()  const { return w_; }
    int height() const { return h_; }

    bool inBounds(int r, int c) const {
        return r >= 0 && r < h_ && c >= 0 && c < w_;
    }

    double at(int r, int c) const {
        return inBounds(r, c) ? cur_[r * w_ + c] : 0.0;
    }

    // Out-of-bounds counts as land so the grid edge never reads as shoreline.
    bool isLand(int r, int c) const {
        return !inBounds(r, c) || land_[r * w_ + c];
    }

    // 3x3 block centred on (r, c). Placing land displaces water outward
    // (a ring splash around the block); erasing leaves a dip that fills in.
    void setLand(int r, int c, bool on) {
        bool changed = false;
        for (int dr = -1; dr <= 1; dr++)
            for (int dc = -1; dc <= 1; dc++) {
                int nr = r + dr, nc = c + dc;
                if (!inBounds(nr, nc)) continue;
                int i = nr * w_ + nc;
                if (land_[i] == on) continue;
                land_[i] = on;
                cur_[i] = prev_[i] = on ? 0.0 : -1.5;
                changed = true;
            }
        if (!changed || !on) return;
        for (int dr = -2; dr <= 2; dr++)
            for (int dc = -2; dc <= 2; dc++) {
                if (std::abs(dr) != 2 && std::abs(dc) != 2) continue;
                int nr = r + dr, nc = c + dc;
                if (inBounds(nr, nc) && !land_[nr * w_ + nc])
                    cur_[nr * w_ + nc] += 2.0;
            }
    }

    // Sharp peak with softer shoulders so rings start round-ish.
    void splash(int r, int c, double power) {
        if (isLand(r, c)) return;
        cur_[r * w_ + c] += power;
        const int dr[] = {-1, 1, 0, 0};
        const int dc[] = {0, 0, -1, 1};
        for (int k = 0; k < 4; k++) {
            int nr = r + dr[k], nc = c + dc[k];
            if (!isLand(nr, nc)) cur_[nr * w_ + nc] += power * 0.4;
        }
    }

    // Land cells are never updated and stay at 0. A land neighbour is read as
    // the cell's own height, so waves reflect off it same-sign like a wall
    // (the grid edge, fixed at 0, reflects inverted).
    void step(double damping) {
        for (int r = 1; r < h_ - 1; r++) {
            for (int c = 1; c < w_ - 1; c++) {
                int i = r * w_ + c;
                if (land_[i]) continue;
                double self = cur_[i];
                auto nb = [&](int j) { return land_[j] ? self : cur_[j]; };
                double next = (nb(i - 1) + nb(i + 1) +
                               nb(i - w_) + nb(i + w_)) / 2.0
                              - prev_[i];
                prev_[i] = next * damping;
            }
        }
        std::swap(cur_, prev_);
    }

private:
    int w_, h_;
    std::vector<double> cur_, prev_;
    std::vector<unsigned char> land_;
};

struct Boat {
    double r, c;
    double vr = 0, vc = 0;

    double speed() const { return std::sqrt(vr * vr + vc * vc); }

    char glyph() const {
        if (speed() < 0.05) return 'B';
        if (std::abs(vc) >= std::abs(vr)) return vc > 0 ? '>' : '<';
        return vr > 0 ? 'v' : '^';
    }

    void update(Water& water, int& wakeTimer) {
        // ride the waves: slide down the local height gradient
        int ir = (int)r, ic = (int)c;
        double gr = water.at(ir + 1, ic) - water.at(ir - 1, ic);
        double gc = water.at(ir, ic + 1) - water.at(ir, ic - 1);
        vr -= gr * 0.012;
        vc -= gc * 0.012;

        vr *= 0.90;
        vc *= 0.90;
        double nr = std::clamp(r + vr, 1.0, (double)water.height() - 2);
        double nc = std::clamp(c + vc, 1.0, (double)water.width() - 2);
        if (water.isLand((int)nr, (int)nc) && !water.isLand(ir, ic)) {
            vr = -vr * 0.5;  // bounce off the shore
            vc = -vc * 0.5;
        } else {
            r = nr;
            c = nc;
        }

        // moving boats periodically disturb the water -> trailing wake
        if (speed() > 0.12 && ++wakeTimer >= 2) {
            wakeTimer = 0;
            water.splash((int)r, (int)c, speed() * 5.0);
        }
    }
};

struct Cell { char ch; int color; };

Cell heightToCell(double h) {
    if (h >  3.0) return {'@', 15};
    if (h >  1.5) return {'^', 51};
    if (h >  0.6) return {'=', 45};
    if (h >  0.2) return {'-', 39};
    if (h > -0.2) return {'~', 33};
    if (h > -1.5) return {'.', 27};
    return {' ', 19};
}

// Build the whole frame into one string and write it in a single syscall.
// Shoreline (land touching water) renders as sand, interior as green.
Cell landCell(const Water& water, int r, int c) {
    bool shore = !water.isLand(r - 1, c) || !water.isLand(r + 1, c) ||
                 !water.isLand(r, c - 1) || !water.isLand(r, c + 1);
    return shore ? Cell{':', 179} : Cell{'#', 28};
}

void render(const Water& water, const std::vector<Boat>& boats,
            size_t activeBoat, bool raining, bool landMode,
            std::string& frame) {
    frame.clear();
    frame += "\x1b[H";

    frame += "\x1b[0;97m click:";
    frame += landMode ? "build  rclick:erase" : "splash";
    frame += "  L:land";
    frame += landMode ? "[ON] " : "[off]";
    frame += "  WASD:drive  b:boat  TAB:switch  SPACE:rain";
    frame += raining ? "[ON] " : "[off]";
    frame += "  q:quit\x1b[K\r\n";

    std::vector<int> overlay(water.width() * water.height(), -1);
    for (size_t i = 0; i < boats.size(); i++)
        overlay[(int)boats[i].r * water.width() + (int)boats[i].c] = (int)i;

    int lastColor = -1;
    for (int r = 0; r < water.height(); r++) {
        for (int c = 0; c < water.width(); c++) {
            int boatIdx = overlay[r * water.width() + c];
            Cell cell = (boatIdx >= 0)
                ? Cell{boats[boatIdx].glyph(),
                       boatIdx == (int)activeBoat ? 226 : 255}
                : water.isLand(r, c) ? landCell(water, r, c)
                                     : heightToCell(water.at(r, c));

            if (cell.color != lastColor) {
                frame += "\x1b[38;5;" + std::to_string(cell.color) + "m";
                lastColor = cell.color;
            }
            frame += cell.ch;
        }
        frame += "\x1b[0m";
        // no newline after the bottom row: it would scroll the HUD off-screen
        if (r < water.height() - 1) frame += "\r\n";
        lastColor = -1;
    }
    write(STDOUT_FILENO, frame.data(), frame.size());
}

int runDemo() {
    Water water(60, 20);
    water.splash(10, 30, 8.0);
    double maxSeen = 0;
    for (int f = 0; f < 60; f++) {
        water.step(0.985);
        maxSeen = std::max(maxSeen, std::abs(water.at(10, 8)));
    }
    for (int r = 0; r < water.height(); r++) {
        for (int c = 0; c < water.width(); c++)
            std::cout << heightToCell(water.at(r, c)).ch;
        std::cout << '\n';
    }
    std::cout << "wave reached col 8 with peak |h| = " << maxSeen << '\n';
    return maxSeen > 0.05 ? 0 : 1;
}

int main(int argc, char** argv) {
    if (argc > 1 && std::string(argv[1]) == "--demo") return runDemo();

    winsize ws{};
    int cols = 80, rows = 24;
    if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &ws) == 0 && ws.ws_col > 0) {
        cols = ws.ws_col;
        rows = ws.ws_row;
    }
    int W = std::max(cols, 20);
    int H = std::max(rows - 1, 10);  // one row reserved for the HUD

    Terminal term;
    if (!term.ok()) {
        std::cerr << "watersim needs an interactive terminal "
                     "(try ./watersim --demo)\n";
        return 1;
    }

    Water water(W, H);
    std::vector<Boat> boats{{H / 2.0, W / 2.0}};
    size_t activeBoat = 0;
    std::vector<int> wakeTimers{0};

    bool raining = false;
    bool landMode = false;
    std::mt19937 rng{std::random_device{}()};
    std::uniform_int_distribution<int> randRow(1, H - 2), randCol(1, W - 2);

    std::string frame;
    frame.reserve(W * H * 12);
    bool running = true;

    while (running) {
        char buf[512];
        ssize_t n = read(STDIN_FILENO, buf, sizeof buf);
        for (ssize_t i = 0; i < n; i++) {
            char ch = buf[i];
            Boat& b = boats[activeBoat];
            switch (ch) {
                case 'q': case 'Q': case 3: running = false; break;
                case 'w': b.vr -= 0.30; break;
                case 's': b.vr += 0.30; break;
                case 'a': b.vc -= 0.45; break;  // cells are taller than wide:
                case 'd': b.vc += 0.45; break;  // boost horizontal thrust
                case ' ': raining = !raining; break;
                case 'l': case 'L': landMode = !landMode; break;
                case '\t': activeBoat = (activeBoat + 1) % boats.size(); break;
                case 'b': case 'B': {
                    int br = randRow(rng), bc = randCol(rng);
                    for (int t = 0; t < 32 && water.isLand(br, bc); t++)
                        br = randRow(rng), bc = randCol(rng);
                    boats.push_back({(double)br, (double)bc});
                    wakeTimers.push_back(0);
                    activeBoat = boats.size() - 1;
                    water.splash((int)boats.back().r, (int)boats.back().c, 6.0);
                    break;
                }
                case '\x1b':  // SGR mouse event: ESC [ < btn ; x ; y M
                    if (i + 2 < n && buf[i + 1] == '[' && buf[i + 2] == '<') {
                        ssize_t j = i + 3;
                        std::string params;
                        while (j < n && buf[j] != 'M' && buf[j] != 'm')
                            params += buf[j++];
                        if (j < n) {
                            int btn, x, y;
                            if (buf[j] == 'M' &&
                                sscanf(params.c_str(), "%d;%d;%d",
                                       &btn, &x, &y) == 3) {
                                // 1-based terminal coords; row 1 is the HUD.
                                // btn & 3: 0 = left, 2 = right (+32 while dragging)
                                if (landMode)
                                    water.setLand(y - 2, x - 1, (btn & 3) != 2);
                                else
                                    water.splash(y - 2, x - 1, 7.0);
                            }
                            i = j;
                        }
                    }
                    break;
            }
        }

        if (raining && rng() % 4 == 0)
            water.splash(randRow(rng), randCol(rng), 2.5);

        for (size_t i = 0; i < boats.size(); i++)
            boats[i].update(water, wakeTimers[i]);

        water.step(0.985);

        render(water, boats, activeBoat, raining, landMode, frame);
        std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }
    return 0;
}
