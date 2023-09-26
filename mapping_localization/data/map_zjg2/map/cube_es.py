import os
from math import sqrt
cube_gt_path = 'cube.txt'
cube_es_path = 'all_objects.txt'
fgt = open(cube_gt_path, mode="r", encoding="utf-8")
fes = open(cube_es_path, mode="r", encoding="utf-8")
lines_gt = fgt.readlines()
lines_es = fes.readlines()
dis_all = 0
dis_x = 0
dis_y = 0
for line in lines_es:
    line = line.strip('\n')
    a = line.split(' ')
    xe = float(a[0])
    ye = float(a[1])
    min_dis = 10000
    dis_x_error = 0
    dis_y_error = 0
    for lineg in lines_gt:
        lineg = lineg.strip('\n')
        b = lineg.split('\t')
        xg = float(b[0])
        yg = float(b[1])
        dis = sqrt((xe-xg)**2+(ye-yg)**2)
        if dis < min_dis:
            min_dis = dis
            dis_x_error = abs(xg-xe)
            dis_y_error = abs(yg-ye)
    if min_dis<2:
        dis_all = dis_all + min_dis
        dis_x = dis_x + dis_x_error
        dis_y = dis_y + dis_y_error
dis_mean = dis_all / len(lines_es)
dis_x_mean = dis_x / len(lines_es)
dis_y_mean = dis_y / len(lines_es)
print("dis_mean:"+str(dis_mean)+" dis_x_mean:"+str(dis_x_mean)+" dis_y_mean:"+str(dis_y_mean))
