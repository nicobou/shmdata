# Monitor frame rate of a shmdata

You need the `pv` utily:
```
sudo apt install pv
```

With `pv` and `sdflow` you can display the frame rate of a shmdata (here `/tmp/video_shmdata`):
```
sdflow /tmp/video_shmdata | pv --line-mode --rate > /dev/null
```

You can get this as a command (`sdfps`) if you copy the following into your `~/.bashrc` file:
```
function sdfps {
        sdflow $1 | pv --line-mode --rate > /dev/null
}
```

Then you can monitor the rate of a shmdata like this:
```
source ~/.bashrc # this reloads the bashrc configuration
sdfps /tmp/video_shmdata
```

