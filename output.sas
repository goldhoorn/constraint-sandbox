begin_version
3
end_version
begin_metric
0
end_metric
20
begin_variable
var0
-1
2
Atom depends(root, mars-camera)
NegatedAtom depends(root, mars-camera)
end_variable
begin_variable
var1
-1
2
Atom depends(root, root)
NegatedAtom depends(root, root)
end_variable
begin_variable
var2
-1
2
Atom depends(testcmp, mars-imu)
NegatedAtom depends(testcmp, mars-imu)
end_variable
begin_variable
var3
-1
2
Atom depends(testcmp, mars-task)
NegatedAtom depends(testcmp, mars-task)
end_variable
begin_variable
var4
-1
2
Atom is-running(mars-altimeter)
NegatedAtom is-running(mars-altimeter)
end_variable
begin_variable
var5
-1
2
Atom is-running(mars-camera)
NegatedAtom is-running(mars-camera)
end_variable
begin_variable
var6
-1
2
Atom is-running(mars-imu)
NegatedAtom is-running(mars-imu)
end_variable
begin_variable
var7
-1
2
Atom is-running(root)
NegatedAtom is-running(root)
end_variable
begin_variable
var8
-1
2
Atom is-running(testcmp)
NegatedAtom is-running(testcmp)
end_variable
begin_variable
var9
1
2
Atom new-axiom@0(mars-camera)
NegatedAtom new-axiom@0(mars-camera)
end_variable
begin_variable
var10
1
2
Atom new-axiom@0(mars-imu)
NegatedAtom new-axiom@0(mars-imu)
end_variable
begin_variable
var11
1
2
Atom new-axiom@0(root)
NegatedAtom new-axiom@0(root)
end_variable
begin_variable
var12
1
2
Atom new-axiom@2(mars-altimeter)
NegatedAtom new-axiom@2(mars-altimeter)
end_variable
begin_variable
var13
1
2
Atom new-axiom@2(mars-camera)
NegatedAtom new-axiom@2(mars-camera)
end_variable
begin_variable
var14
1
2
Atom new-axiom@2(mars-imu)
NegatedAtom new-axiom@2(mars-imu)
end_variable
begin_variable
var15
0
2
Atom new-axiom@2(mars-task)
NegatedAtom new-axiom@2(mars-task)
end_variable
begin_variable
var16
1
2
Atom new-axiom@2(root)
NegatedAtom new-axiom@2(root)
end_variable
begin_variable
var17
1
2
Atom new-axiom@2(testcmp)
NegatedAtom new-axiom@2(testcmp)
end_variable
begin_variable
var18
1
2
Atom new-axiom@3()
NegatedAtom new-axiom@3()
end_variable
begin_variable
var19
1
2
Atom new-axiom@4()
NegatedAtom new-axiom@4()
end_variable
0
begin_state
1
0
0
0
0
1
0
0
0
0
0
0
1
1
1
1
1
1
0
0
end_state
begin_goal
3
7 0
18 1
19 1
end_goal
8
begin_operator
startt root mars-camera
0
2
0 0 -1 0
0 5 1 0
0
end_operator
begin_operator
startt testcmp mars-imu
0
2
0 2 -1 0
0 6 1 0
0
end_operator
begin_operator
stop mars-altimeter
0
1
0 4 -1 1
0
end_operator
begin_operator
stop mars-camera
1
9 1
2
0 0 -1 1
0 5 -1 1
0
end_operator
begin_operator
stop mars-imu
1
10 1
2
0 2 -1 1
0 6 -1 1
0
end_operator
begin_operator
stop mars-task
0
1
0 3 -1 1
0
end_operator
begin_operator
stop root
1
11 1
2
0 1 -1 1
0 7 -1 1
0
end_operator
begin_operator
stop testcmp
0
1
0 8 -1 1
0
end_operator
11
begin_rule
0
15 1 0
end_rule
begin_rule
1
4 1
12 1 0
end_rule
begin_rule
3
5 0
6 0
7 0
19 0 1
end_rule
begin_rule
1
5 1
13 1 0
end_rule
begin_rule
1
6 1
14 1 0
end_rule
begin_rule
1
7 1
16 1 0
end_rule
begin_rule
1
8 1
17 1 0
end_rule
begin_rule
6
12 0
13 0
14 0
15 0
16 0
17 0
18 0 1
end_rule
begin_rule
0
13 1 0
end_rule
begin_rule
0
16 1 0
end_rule
begin_rule
0
14 1 0
end_rule
