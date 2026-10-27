# TCU test workspace (PlatformIO + ESP-IDF, C)

1. Open `src/main/main.c`.
2. Leave one `test_xxx_run();` line uncommented.
3. Click Upload, then Monitor (115200).

```
src/
  main/main.c         picks the test
  pins/pins.h         all pin numbers
  tests/<name>/       test_<name>.c + test_<name>.h
```

Only `current_sensor` is written. The other tests are stubs with a TODO comment at the top of each file.
New `.c` files under `src/` are compiled automatically. Not built with the ESP-IDF toolchain here.
