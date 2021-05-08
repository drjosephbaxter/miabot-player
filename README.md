# Miabot Player Driver

This is a nostalgic commit of the Miabot player driver I wrote from my PhD.
Worked with Player 2.1 (http://playerstage.sourceforge.net/)

## Build
To compile Miabot plugin type the following commands at the command prompt.

```
$ g++ `pkg-config --cflags playercore` -o miabot.so -shared miabot.cc miabot_commands.cc miabot_params.cc `pkg-config --libs playercore` -lbluetooth
```

## Run

```
$ player miabot_example.cfg
```
