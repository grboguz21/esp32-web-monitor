# ESP32 Web Monitor

A small dashboard I made to keep an eye on my ESP32 from the browser over USB.
It's an early draft. I'm planning to use it as the starting point for a motor controller project.

![Dashboard](docs/dashboard.png)

What it does right now:

- connects straight from Chrome or Edge over USB, no app to install
- shows if the board is alive, how much memory it has left, how fast the main loop runs and why it last restarted
- has a clickable pinout of the DOIT ESP32 DevKit V1, so I don't have to look up which pin does what every time

![Board pinout](docs/board-pinout.png)

## Trying it out

Flash the firmware with PlatformIO (`pio run -t upload`), open `dashboard/index.html` in Chrome, click **Add port…**, pick the board and hit **Connect**.

Close the PlatformIO serial monitor first, otherwise the port is busy.

On Linux, if the board doesn't show up at all, `brltty` is probably grabbing it. `sudo apt remove brltty` fixed it for me.

## License

MIT
