# 방 사이 이동 경로 최적화 (평균 가중평균택시거리 기준)
import numpy as np
import copy
import random as rand

class StartRoom:
    """
    시작 방의 특징을 정의하는 클래스입니다
    endList : 이 방이 찾는 end의 index(end 번호)와 weight(가중치)를 갖는 dict의 list입니다
    가중치는 합이 1이 되도록 자동으로 정규화됩니다.
    """
    def __init__(self, endList):
        sum = 0 # weight의 합이 1이 되도록 정규화
        for end in endList:
            sum += end['weight']
        for end in endList:
            end['weight'] /= sum

        self.endList = endList

def CalcMeanDistance(roomset, stair):
    """
    방과 계단의 배치를 받아 각 출발 방에서 도착 방까지의 거리의 평균을 계산합니다
    """ #이거 만드는데 4시간 걸렸다 ㅁㄴㅇㄹ
    rList = []
    eList = []
    for y in range(len(roomset)):
        for x in range(len(roomset[0])):
            if(isinstance(roomset[y][x], StartRoom)): rList.append({'room': roomset[y][x], 'pos': (x,y)})
            elif(roomset[y][x] > 0): eList.append({'index': roomset[y][x], 'pos': (x,y)})

    distance = 0
    for room in rList:
        distanceR = 0
        for end in room['room'].endList:
            targetStair = findStairFromEnd(searchEndfromeList(end['index'], eList)['pos'][0], stair)
            if(isinstance(targetStair, int)): distanceR += weightedTaxiDistance(room['pos'], targetStair, searchEndfromeList(end['index'], eList)['pos']) * end['weight']
            else:
                if(targetStair[0] == -1): distanceR += weightedTaxiDistance(room['pos'], targetStair[1], searchEndfromeList(end['index'], eList)['pos']) * end['weight']
                elif(targetStair[1] == float('inf')): distanceR += weightedTaxiDistance(room['pos'], targetStair[0], searchEndfromeList(end['index'], eList)['pos']) * end['weight']
                else: distanceR += min(weightedTaxiDistance(room['pos'], targetStair[0], searchEndfromeList(end['index'], eList)['pos']), weightedTaxiDistance(room['pos'], targetStair[1], searchEndfromeList(end['index'], eList)['pos'])) * end['weight']
        distance += distanceR
    distance /= len(rList)
    return distance

weightedTaxiDistance = lambda posr, xs, pose: 7*(abs(posr[0]-xs) + abs(pose[0]-xs)) + 16*abs(posr[1]-pose[1])
# 걷기와 계단오르기의 운동 강도(MET)비 3.5 : 8을 반영한 '출발 방 -> 계단 -> 도착 방'의 경로 거리
# MET는 같은 사람이 같은 시간당 소비하는 칼로리 계수

def getIndexfromrList(pos, rList):
    """
    rList에서 특정 pos의 원소를 찾아 그 원소를 리턴한다
    없으면 None을 리턴한다
    """
    for i in range(len(rList)):
        if(rList[i]['pos'][0] == pos[0] and rList[i]['pos'][1] == pos[1]): return rList[i]
    return None # 없는 경우

def searchEndfromeList(index, eList):
    """
    eList에서 특정 index가 포함된 원소를 찾아 그 원소를 리턴한다
    없으면 None을 리턴한다
    """
    for i in range(len(eList)):
        if(eList[i]['index'] == index): return eList[i]
    return None # 없는 경우

def findStairFromEnd(x, stair):
    """
    도착 방의 index와 stair 리스트를 주면 방 근처의 계단 x좌표(작은쪽, 큰쪽)를 찾아주는 함수
    이진 탐색을 이용하기 때문에 stair는 오름차순 정렬되어 있어야 함
    """
    check = len(stair) // 2
    findedStair = [-1, float("inf")]
    searchingIndex = [0, len(stair) - 1]
    for _ in range(len(stair) + 1):
        if(searchingIndex[0] > searchingIndex[1]): return findedStair
        elif(stair[check] == x): return x
        elif(stair[check] > x):
            findedStair[1] = stair[check]
            if(check <= 0): return findedStair
            searchingIndex[1] = check - 1
            check = (searchingIndex[0] + check - 1) // 2
        else:
            findedStair[0] = stair[check]
            if(check >= len(stair) - 1): return findedStair
            searchingIndex[0] = check + 1
            check = (searchingIndex[1] + check + 1) // 2

def optimizeRoomset1(roomset, stair):
    opRoomset = copy.deepcopy(roomset)
    rList = []
    eList = []
    for y in range(len(opRoomset)):
        for x in range(len(opRoomset[0])):
            if(isinstance(opRoomset[y][x], StartRoom)): rList.append({'room': opRoomset[y][x], 'pos': [x,y]})
            elif(opRoomset[y][x] > 0): eList.append({'index': opRoomset[y][x], 'pos': [x,y]})
    for count in range(100000):
        tempRoomset = copy.deepcopy(opRoomset)
        distance = [float('inf'), float('inf'), float('inf'), float('inf'), float('inf')]
        distance[0] = CalcMeanDistance(opRoomset, stair)

        #오버/언더플로우 -> 롤 시스템 -> 오류
        if(rList[count % len(rList)]['pos'][0] - 1 > -1 and (isinstance(opRoomset[rList[count % len(rList)]['pos'][1]][rList[count % len(rList)]['pos'][0] - 1], StartRoom) or opRoomset[rList[count % len(rList)]['pos'][1]][rList[count % len(rList)]['pos'][0] - 1] == 0)):
            temp = opRoomset[rList[count % len(rList)]['pos'][1]][rList[count % len(rList)]['pos'][0]]
            opRoomset[rList[count % len(rList)]['pos'][1]][rList[count % len(rList)]['pos'][0]] = opRoomset[rList[count % len(rList)]['pos'][1]][rList[count % len(rList)]['pos'][0] - 1]
            opRoomset[rList[count % len(rList)]['pos'][1]][rList[count % len(rList)]['pos'][0] - 1] = temp
            distance[1] = CalcMeanDistance(opRoomset, stair)
            temp = opRoomset[rList[count % len(rList)]['pos'][1]][rList[count % len(rList)]['pos'][0]]
            opRoomset[rList[count % len(rList)]['pos'][1]][rList[count % len(rList)]['pos'][0]] = opRoomset[rList[count % len(rList)]['pos'][1]][rList[count % len(rList)]['pos'][0] - 1]
            opRoomset[rList[count % len(rList)]['pos'][1]][rList[count % len(rList)]['pos'][0] - 1] = temp
        if(rList[count % len(rList)]['pos'][0] + 1 < len(opRoomset[0]) and (isinstance(opRoomset[rList[count % len(rList)]['pos'][1]][rList[count % len(rList)]['pos'][0] + 1], StartRoom) or opRoomset[rList[count % len(rList)]['pos'][1]][rList[count % len(rList)]['pos'][0] + 1] == 0)):
            temp = opRoomset[rList[count % len(rList)]['pos'][1]][rList[count % len(rList)]['pos'][0]]
            opRoomset[rList[count % len(rList)]['pos'][1]][rList[count % len(rList)]['pos'][0]] = opRoomset[rList[count % len(rList)]['pos'][1]][rList[count % len(rList)]['pos'][0] + 1]
            opRoomset[rList[count % len(rList)]['pos'][1]][rList[count % len(rList)]['pos'][0] + 1] = temp
            distance[2] = CalcMeanDistance(opRoomset, stair)
            temp = opRoomset[rList[count % len(rList)]['pos'][1]][rList[count % len(rList)]['pos'][0]]
            opRoomset[rList[count % len(rList)]['pos'][1]][rList[count % len(rList)]['pos'][0]] = opRoomset[rList[count % len(rList)]['pos'][1]][rList[count % len(rList)]['pos'][0] + 1]
            opRoomset[rList[count % len(rList)]['pos'][1]][rList[count % len(rList)]['pos'][0] + 1] = temp
        if(rList[count % len(rList)]['pos'][1] - 1 > -1 and (isinstance(opRoomset[rList[count % len(rList)]['pos'][1] - 1][rList[count % len(rList)]['pos'][0]], StartRoom) or opRoomset[rList[count % len(rList)]['pos'][1] - 1][rList[count % len(rList)]['pos'][0]] == 0)):
            temp = opRoomset[rList[count % len(rList)]['pos'][1]][rList[count % len(rList)]['pos'][0]]
            opRoomset[rList[count % len(rList)]['pos'][1]][rList[count % len(rList)]['pos'][0]] = opRoomset[rList[count % len(rList)]['pos'][1] - 1][rList[count % len(rList)]['pos'][0]]
            opRoomset[rList[count % len(rList)]['pos'][1] - 1][rList[count % len(rList)]['pos'][0]] = temp
            distance[3] = CalcMeanDistance(opRoomset, stair)
            temp = opRoomset[rList[count % len(rList)]['pos'][1]][rList[count % len(rList)]['pos'][0]]
            opRoomset[rList[count % len(rList)]['pos'][1]][rList[count % len(rList)]['pos'][0]] = opRoomset[rList[count % len(rList)]['pos'][1] - 1][rList[count % len(rList)]['pos'][0]]
            opRoomset[rList[count % len(rList)]['pos'][1] - 1][rList[count % len(rList)]['pos'][0]] = temp
        if(rList[count % len(rList)]['pos'][1] + 1 < len(opRoomset) and (isinstance(opRoomset[rList[count % len(rList)]['pos'][1] + 1][rList[count % len(rList)]['pos'][0]], StartRoom) or opRoomset[rList[count % len(rList)]['pos'][1] + 1][rList[count % len(rList)]['pos'][0]] == 0)):
            temp = opRoomset[rList[count % len(rList)]['pos'][1]][rList[count % len(rList)]['pos'][0]]
            opRoomset[rList[count % len(rList)]['pos'][1]][rList[count % len(rList)]['pos'][0]] = opRoomset[rList[count % len(rList)]['pos'][1] + 1][rList[count % len(rList)]['pos'][0]]
            opRoomset[rList[count % len(rList)]['pos'][1] + 1][rList[count % len(rList)]['pos'][0]] = temp
            
            distance[4] = CalcMeanDistance(opRoomset, stair)
            temp = opRoomset[rList[count % len(rList)]['pos'][1]][rList[count % len(rList)]['pos'][0]]
            opRoomset[rList[count % len(rList)]['pos'][1]][rList[count % len(rList)]['pos'][0]] = opRoomset[rList[count % len(rList)]['pos'][1] + 1][rList[count % len(rList)]['pos'][0]]
            opRoomset[rList[count % len(rList)]['pos'][1] + 1][rList[count % len(rList)]['pos'][0]] = temp

        shortestDistanceIndex = 0
        for i in range(5):
            if(distance[i] == distance[shortestDistanceIndex]): shortestDistanceIndex = rand.choice([i,shortestDistanceIndex])
            elif(distance[i] < distance[shortestDistanceIndex]): shortestDistanceIndex = i

        if(shortestDistanceIndex == 1):
            temp = opRoomset[rList[count % len(rList)]['pos'][1]][rList[count % len(rList)]['pos'][0]]
            opRoomset[rList[count % len(rList)]['pos'][1]][rList[count % len(rList)]['pos'][0]] = opRoomset[rList[count % len(rList)]['pos'][1]][rList[count % len(rList)]['pos'][0] - 1]
            opRoomset[rList[count % len(rList)]['pos'][1]][rList[count % len(rList)]['pos'][0] - 1] = temp
            rList[count % len(rList)]['pos'][0] -= 1
            a = getIndexfromrList(rList[count % len(rList)]['pos'], rList)
            if(isinstance(a, StartRoom)): a['pos'][0] += 1
        elif(shortestDistanceIndex == 2):
            temp = opRoomset[rList[count % len(rList)]['pos'][1]][rList[count % len(rList)]['pos'][0]]
            opRoomset[rList[count % len(rList)]['pos'][1]][rList[count % len(rList)]['pos'][0]] = opRoomset[rList[count % len(rList)]['pos'][1]][rList[count % len(rList)]['pos'][0] + 1]
            opRoomset[rList[count % len(rList)]['pos'][1]][rList[count % len(rList)]['pos'][0] + 1] = temp
            rList[count % len(rList)]['pos'][0] += 1
            a = getIndexfromrList(rList[count % len(rList)]['pos'], rList)
            if(isinstance(a, StartRoom)): a['pos'][0] -= 1
        elif(shortestDistanceIndex == 3):
            temp = opRoomset[rList[count % len(rList)]['pos'][1]][rList[count % len(rList)]['pos'][0]]
            opRoomset[rList[count % len(rList)]['pos'][1]][rList[count % len(rList)]['pos'][0]] = opRoomset[rList[count % len(rList)]['pos'][1] - 1][rList[count % len(rList)]['pos'][0]]
            opRoomset[rList[count % len(rList)]['pos'][1] - 1][rList[count % len(rList)]['pos'][0]] = temp
            rList[count % len(rList)]['pos'][1] -= 1
            a = getIndexfromrList(rList[count % len(rList)]['pos'], rList)
            if(isinstance(a, StartRoom)): a['pos'][1] += 1
        elif(shortestDistanceIndex == 4):
            temp = opRoomset[rList[count % len(rList)]['pos'][1]][rList[count % len(rList)]['pos'][0]]
            opRoomset[rList[count % len(rList)]['pos'][1]][rList[count % len(rList)]['pos'][0]] = opRoomset[rList[count % len(rList)]['pos'][1] + 1][rList[count % len(rList)]['pos'][0]]
            opRoomset[rList[count % len(rList)]['pos'][1] + 1][rList[count % len(rList)]['pos'][0]] = temp
            rList[count % len(rList)]['pos'][1] += 1
            a = getIndexfromrList(rList[count % len(rList)]['pos'], rList)
            if(isinstance(a, StartRoom)): a['pos'][1] -= 1

    return opRoomset








rList1 = []
rList2 = []
rList3 = []
for _ in range(5):
    rList1.append(StartRoom([{'index': 1, 'weight': 1}]))
    rList2.append(StartRoom([{'index': 2, 'weight': 1}]))
    rList3.append(StartRoom([{'index': 3, 'weight': 1}]))
# 0은 출발 방이 들어갈 수 있는 빈칸
# StartRoom 인스턴스는 출발 방
# 양수는 도착 방
# -1은 출발 방이 들어갈 수 없는 빈칸
roomset = [
    rList1,
    rList2,
    rList3,
    [0,2,0,1,0,],
    [0,0,0,0,3,],
]
stair = [0,2,4]
print('----------------')
print(CalcMeanDistance(roomset,stair))
opRoomset = optimizeRoomset1(roomset,stair)
roomVisual = []
for y in range(len(opRoomset)):
    arr = []
    for x in range(len(opRoomset[0])):
        a = opRoomset[y][x]
        if(isinstance(a, int)): arr.append(a)
        elif(isinstance(a, StartRoom)): arr.append('p' + str(a.endList[0]['index']))
    roomVisual.append(arr)
print(roomVisual)
print(CalcMeanDistance(opRoomset, stair))