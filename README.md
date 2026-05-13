# SC-OBC Safety Processor

This is the firmware for SC-OBC Safety Processor.

## Getting started

Create a new west workspace from this repository:

```shell
$ west init -m https://github.com/spacecubics/scobc-sp.git scobc-sp-workspace
$ cd scobc-sp-workspace
$ west update
```

## Build

### SC-OBC Module V1

```shell
$ west build -p always -b scobc_v1/miv scobc-sp
```
