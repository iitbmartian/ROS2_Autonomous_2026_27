#!/bin/bash
# Serves dashboard/www on the Tailscale address only, port 8080, so the campus LAN cannot reach it.
# Run from cron every 5 min: starts the server once Tailscale is up, restarts it after a reboot.
D=$HOME/.ros/rover_3d_nav/dashboard
IP=$(tailscale ip -4 2>/dev/null | head -1)
[ -z "$IP" ] && exit 0
pgrep -f "http[.]server 8080 --bind $IP" > /dev/null && exit 0
pkill -f "http[.]server 8080" 2>/dev/null
nohup python3 -m http.server 8080 --bind "$IP" -d "$D/www" > "$D/serve.log" 2>&1 &
