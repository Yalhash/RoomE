#!/bin/bash
cmake -S . -B build/ && cp -r assets build/assets && cd build && make -j4 && ./roomE
