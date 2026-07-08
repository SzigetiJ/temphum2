## Temperature and Humidity

![GitHub](https://img.shields.io/github/license/SzigetiJ/temphum2)
<!--![C/C++ CI](https://github.com/SzigetiJ/temphum2/workflows/C/C++%20CI/badge.svg)-->
[![GitHub code size](https://img.shields.io/github/languages/code-size/SzigetiJ/temphum2)](https://github.com/SzigetiJ/temphum2)
![GitHub repo size](https://img.shields.io/github/repo-size/SzigetiJ/temphum2)
![GitHub commit activity](https://img.shields.io/github/commit-activity/y/SzigetiJ/temphum2)
![GitHub issues](https://img.shields.io/github/issues/SzigetiJ/temphum2)
![GitHub closed issues](https://img.shields.io/github/issues-closed/SzigetiJ/temphum2)

This is a small **ESP32 application** displaying real-time temperature and relative humidity information
on a 4-digit 7 segment display.
The applied temperature and humidity sensor is **DHT22**,
The 4-digit 7 segment display is **TM1637**.
The application is based on [ESP32Basic](https://github.com/SzigetiJ/esp32basic) lightweight framework
(i.e., it does **not** depend on *ESP-IDF*).

Note, both projects (esp32basic and this one) are in development phase,
and the version-dependency is not set yet.

## Hardware setup

The required components, wiring, etc. are described in [src/README.md](src).

## Software Installation

### Get the source

Either clone the git source or download and extract the zip.
Cloning git source is preferred. It is easier to update.

### Autotools preparation

First, you need to run `aclocal`.

Next, run `autoconf`.

Finally, run `automake --add-missing`.

```sh
aclocal
autoconf
automake --add-missing
```

### Configure & Install

The [INSTALL](INSTALL) file describes the general way of the installation of an automake project.

Here, we present the specialities, so the
```sh
./configure
make
make install
```
command sequence is slightly modified.

* Usually, it is a good idea to do [VPATH Builds](https://www.gnu.org/software/automake/manual/html_node/VPATH-Builds.html).
We encourage you to create at least a _performance_ build in `dist/perf` (with `-O2` compiler option)
and a _debug_ build in `dist/debug` (with `-O0 -g` compiler options).

* As the target platform is ESP32, we have to use the `xtensa-elf-esp` toolchain.
In order to use the `xtensa-esp-elf` toolchain, you have to call the configure script with option
`--host=xtensa-esp32-elf`. Also set option `host_alias=xtensa-esp32-elf`.
Further reading: [online manual](https://www.gnu.org/savannah-checkouts/gnu/autoconf/manual/autoconf-2.70/html_node/Hosts-and-Cross_002dCompilation.html#Hosts-and-Cross_002dCompilation).

* This project depends on `esp32basic` (libs and headers).
You have to specify the installation location of `esp32basic`:
`--with-e32bdir=PATH_TO_ESP32BASIC_INSTALLATION`.

* Currently, some commonly used compiler options are not configured automatically.
Thus you have to configure them manually (define in `CFLAGS` for the `configure` script):
`-Werror -nostdlib -mlongcalls -std=c11 -flto`

My performance VPATH build configuration 'script' looks like this:
```
 mkdir -p dist/perf
 cd dist/perf
 ../../configure --host=xtensa-esp32-elf 'CFLAGS=-O2 -Wall -Werror -nostdlib -mlongcalls -std=c11 -flto' host_alias=xtensa-esp32-elf --with-e32bdir=/home/szigeti/local/esp32
```

Build is started by command
```
 make
```
within `dist/perf`. If everything goes well, it produces `src/temphum.bin` relative to `dist/perf`.

If your ESP32 is bound with UART adapter, e.g., CP2102, like in NodeMCU-ESP32S, installation is simple.
To install (i.e., write the binary to the ESP32 board) via UART, you can use `scripts/flash.sh`:
```
 ../../scripts/flash.sh src/temphum.bin
```
