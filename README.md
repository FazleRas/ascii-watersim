# watersim

An interactive ASCII water simulator for your terminal, written in C++17 with zero dependencies. Click to splash, drive boats around, watch the wakes ripple.

```
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
~..--~~.~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~..~~
~-~~-~..~~-~~.....~~~~~~-~-~-~-~-~~~~~.....~~~-~..=~
~~~...~-=-~~......~.~.~.~.~.~.~.~.~.~.~.....~---~.~~
~.~~..~-=--~~~~~............>..............~--=-~.~~
~~~...~--=-~~~~~~..........................~-=--~.~~
~~..~~~----~~.......~.~.~.~.~.~.~.~.~.......~---~~~~
~~.~=-.~~~~....~~~~~~~~~~~~~~~~~~~~~~~~~....~~~..~~~
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
```

## Features

- **Click to splash** — real mouse support in the terminal (SGR mouse reporting); drag to carve streams
- **Drivable boats** — WASD to steer; moving boats leave a rippling wake, and because the wave source moves, the rings compress ahead and stretch behind (a visible Doppler pattern)
- **Boats ride the waves** — splash next to a parked boat and it gets shoved away
- **Multiple boats** — spawn as many as you like and switch between them
- **Rain mode** — random droplets across the whole pond
- 256-color height-mapped rendering, ~30 FPS, single flicker-free write per frame

## Build & run

Requires a C++17 compiler and a POSIX terminal (macOS / Linux).

```sh
g++ -std=c++17 -O2 main.cpp -o watersim
./watersim
```

`./watersim --demo` runs a non-interactive smoke test (one splash, 60 physics steps, prints the resulting frame).

## Controls

| Key         | Action                          |
|-------------|---------------------------------|
| mouse click | splash (drag for streams)       |
| `W A S D`   | drive the active boat           |
| `b`         | spawn a new boat                |
| `TAB`       | switch active boat              |
| `SPACE`     | toggle rain                     |
| `q`         | quit                            |

## How it works

The water is a height field over two buffers. Each frame, every cell becomes

```
next = (sum of 4 neighbours) / 2 - previous
next *= damping
```

The `- previous` term gives waves momentum so ripples travel outward instead of diffusing in place, and damping (`0.985`) lets the pond settle. The terminal is put into raw mode via `termios` for unbuffered keyboard + mouse input, and each frame is rendered into a single string and written with one syscall to avoid flicker.
