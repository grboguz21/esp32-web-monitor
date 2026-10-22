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

1. Open this folder in VS Code with the PlatformIO extension installed.
2. Plug the ESP32 in with a USB cable and click **Upload** (the → arrow in the bottom bar).
3. If the PlatformIO serial monitor is open, close it. Only one program can use the port.
4. Open `dashboard/index.html` in Chrome. Double-clicking usually works. If it opens in another browser, right-click the file and choose **Open with → Google Chrome**, or just drag it into a Chrome window.
5. Click **Add port…**, pick your board from the list (mine shows up as *USB Serial*) and click **Connect**.
6. The dot in the top-left turns green when the board is talking to the page.

It only works in Chrome or Edge. Firefox and Safari don't support Web Serial.
Before uploading new code, click **Disconnect** so the port is free again.

On Linux, if the board doesn't show up at all, `brltty` is probably grabbing it. `sudo apt remove brltty` fixed it for me.

## License

MIT
