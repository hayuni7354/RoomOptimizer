우리 9반 교실은 위치가 나름 괜찮았다.
조금만 가면 음악실이 있고, 한 층만 내려가면 일본어실과 급식실이 있어 편리하였다.
하지만 물2, 인공지능 기초를 듣기 위해서는 불편을 겪어야 했는데, 왜냐하면 물리실과 컴퓨터실이 모두 2층에 있었기 때문이였다.
그래서 학교 구조를 통해 최적의 교실 배치를 찾는 프로그램을 만들어 보기로 하였다.

우선 교실 배치를 최적화할 기준은 평균 피로도의 최솟값을 이용하기로 하였다.
이때 피로도는 층 내에서 이동시 1칸당 7, 층간 이동시 1칸당 16만큼을 주기로 하였다.
이는 MET라는 운동 강도를 나타내는 수치(같은 사람 같은 시간 운동시 소모되는 에너지의 계수)의 비율을 따른 것이다 (실제 값은 3.5, 8이지만 정수로 맞추기 위해 각각 2배)
또한 각 반의 선택 과목이 모두 일치하지는 않을 것이므로, 목표 지점이 여럿일 경우에는 각 지점마다 가중치를 다르게 할 수 있도록 하였다.

또한 층간 이동을 하려면 계단이 있어야 하므로, 계단 배치까지 고려하여 경로를 찾고, 피로도를 계산하였다.

이제 어떻게 배치를 최적화할 지 생각해보았다.
교실 배치 최적화? 최단 피로도의 평균을 미분하면 되지 않을까?
https://darkpgmr.tistory.com/149 << 이것처럼 다양한 최적화 방식을 쓸 수도 있겠다.
그런데... 교실 배치는 연속적이지 않아서 (벽을 옮기면서 교실 위치를 조정할 수 없으니) 미분할 수 없다는 문제가 있었다!

하지만 여전히 미분의 아이디어는 사용 가능하다.
미분을 통한 최적화의 기본 아이디어는 미분으로 방향을 정한 후, 그 방향으로 조금 나아가는 것이기 때문이다.
따라서 평균 피로도가 최적화되는 방향을 찾아, 조금 변화시키면 최적화가 될 것이라고 생각했다.

따라서 조금만 변화시키기 위해 교실(코드에서는 일반화해서 '방'이라고 표현한다.)을 하나만 골라(다만 모든 방이 골고루 골라지도록 해야 할 것이다) 근처 4개의 교실(상하좌우)와 자리를 바꿔 보고,
각각의 최단 피로도의 평균을 계산하여 최소가 되는 것을 선택해 나아가는 방식을 택했다.
이 방법으로 optimizeRoomset1 함수를 얻었다.

이 함수를 이용해 간단한 5x5의 모형 학교(우리 학교와는 구조가 다르지만 꽤 복잡하다)를 이용해 최적화 능력을 확인해보았다. 모형의 기호에 대한 설명은 아래와 같다.
0 : 교실이 들어올 수 있는 빈 곳 / -1 : 교실이 들어올 수 없는 빈 곳 / n : 도착점(우리 학교로 치면 특별실) / pn : 출발점(우리 학교로 치면 각반 교실) / pn의 학생은 모두 n으로 가야 한다고 가정하였다.
최적화 전 모형 학교의 방 구조는 다음과 같고, 계단은 1열, 3열, 5열에 있다.
p1 p1 p1 p1 p1
p2 p2 p2 p2 p2
p3 p3 p3 p3 p3 / 평균 피로도 50.4
 0  2  0  1  0
 0  0  0  0  3

optimizeRoomset1을 이용하여 1000번 조정하여 최적화된 구조(가장 잘 나온 것)는 다음과 같다.
0   0 p1  0  0
p2  0 p1 p1 p3
p2 p2 p1 p3 p1 / 평균 피로도 약 27.7
p2  2 p2  1 p3
 0  0 p3 p3  3

기존의 배치보다는 훨씬 나아졌으나, p3가 3층과 4층에 남아 있고 p1이 5층에 남아 있는 등 아직도 개선되야 할 점이 많이 보인다.
또한 평균적으로는 피로도 30정도인 구조를 주로 제공하며, 이런 구조들은 최적화된 결과와는 거리가 멀다.

이후 위 사이트에서 'Line Search 방법'이라는 미분을 활용한 최적화 기법에서 추가적인 영감을 얻게 되었다.
Line Search 중 Backtracking line search 방법은 접선을 따라 이동할 때 우선 최대한 멀리 가본 후,
충분히 함수값이 감소하지 않는다면(최소값 문제일 때) 절반만 가보고 함수값을 비교하는 과정을 반복하고,
충분히 감소하였다면 그 지점으로 이동 후 처음부터 반복하는 기법이다.
이 방법은 함수의 연속과 미분가능성이 필요하지 않기 때문에 충분히 적용시킬 수 있을 것 같아서, 이를 적용하여 optimizerRoomset2 함수를 얻었다.

optimizeRoomset2을 이용하여 1000번 조정하여 최적화된 구조(가장 잘 나온 것)는 다음과 같다.
 0  0  0  0  0
 0 p1 p1 p2 p1
p2 p1 p1 p2 p3 / 평균 피로도 약 25.3
p2  2 p2  1 p3
 0 p3 p3 p3  3

최선의 경우에는 optimizeRoomset1보다 나은 구조를 제공하나,
평균적으로는 여전히 피로도 30정도인 구조를 주로 제공하며, 근본적인 해결책이 되지는 않음을 알 수 있다.
그리고 이 구조도 2번째 열과 4번째 열의 p1과 p2를 바꾸면 피로도를 더 낮출 수 있다는 점에서, 현재 구조가 최고의 구조가 아니라는 점도 문제로 남아있다.

아쉽게도 이번 활동으로 유의미한 결과를 얻지는 못했지만, 불연속적인 구조도 미분을 이용한 최적화의 아이디어를 적용할 수 있다는 것을 알게 되었고,
이를 활용하기 위해서는 미분을 이용한 최적화보다 많은 개량이 필요할 수도 있음을 알게 되었다.
또한 인공지능 수학 동아리에서 배운 내용을 변형하여 적용해보며 그 유용성을 실감하게 되고, '그런게 있다' 정도로만 알고 있던 택시 거리를 실제로 사용해보는 기회가 되었다.
또한 이번 활동의 실패 원인은 이 방식을 사용했을 때 배치 구조가 금방 안정화되어 버려서 최적이 아닌 안정한 구조(극솟값과 비슷한 느낌)를 탈출할 방법을 고안하지 못했기 때문이였던 것 같다.
추후에는 이 알고리즘을 개량하여, 어느 정도 유의미한 결과를 도출해 보고 싶고, 이를 실제 우리 학교의 구조에도 적용해 보고 싶다. (계단과 방의 위치만 잘 대입하면 된다)

구현 과정에서 배운거나 적용한 점
1. 방의 양쪽에 있는 계단의 좌표를 알기 위해 이진 정렬 알고리즘을 이용하였다.
2. 나머지 연산자를 이용해 균등하게 각각의 방에 이동 기회를 주는 알고리즘 제작하면서 나머지 개념의 수학적 흥미로움에 대해 생각해보게 되었다.
3. 가장 고치기 어려웠던 버그는 optimizeRoomset2 구현시 짧은 이동에 의해 긴 이동도 검사를 해보게 되지만 짧은 이동이 평균 피로도에 영향을 주지 않아 얼만큼 거리를 이동하는지에 대한 정보가 남아버리게 되고,
4. 그대로 적용되어 그 장소에 원래 있던 도착점 방이 출발점 방과 위치가 바뀌는 버그였다.
5. 


