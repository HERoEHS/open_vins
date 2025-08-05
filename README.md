### 실제 로봇에서 실행
```
ros2 launch ov_msckf subscribe.launch.py config:=edie use_sim_time:=false
```
### 시뮬레이션상에서 실행
```
ros2 launch ov_msckf subscribe.launch.py config:=edie use_sim_time:=true
```
