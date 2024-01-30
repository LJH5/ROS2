# Joystick을 이용한 turtlebot 제어

### 기능
1. LB 버튼을 누르면 방향 패드로 조작이 가능

2. A, B, X, Y 버튼에 미리 좌표를 지정하고 해당 버튼 누를 시 지정된 위치로 이동
   - PoseStamped() action을 이용해서 제어

3. RB 버튼 누르면 이동을 취소하는 기능
   - PoseStamped() action을 이용해서 제어

### 후기
- action을 사용하여 터틀봇을 제어하기 위해 연습함
- 비동기로 보내서 응답을 기다리지 않아도 작동하도록 함
- feedback을 이용하여 상태를 체크함