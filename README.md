# Kids Arduino

To get setup

1. use the arduino container to get the required deps.
2. try to comile, you might need to install deps for the project:

```
arduino-cli compile --export-binaries --build-path=build/

arduino-cli lib search IRremote
arduino-cli lib install IRremote
```
