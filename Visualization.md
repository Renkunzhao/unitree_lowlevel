# Remote Visualization

## SSH -X/Y
```bash
ssh -Y unitree@
sudo apt install x11-apps
xclock
```

## NoMachine
```bash
sudo service gdm3 stop
sudo init 3
sudo /etc/NX/nxserver --restart
```

## Docker
GUI visualization inside the container is still not working. Please run unitree_mujoco directly on the host machine instead of inside Docker.
```bash
echo $DISPLAY
echo $XAUTHORITY

```