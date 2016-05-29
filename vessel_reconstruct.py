#!/usr/bin/env python
# coding=utf-8

import math
import time
import vtk
import sys


# parse file which represent vessel
def do_parse_vessel_file(path, x, y, z):
    centerline = []
    r_spacing = x
    f = open(path, 'r')
    for line in f:
        line = line.split()
        if len(line) is 5:
            radius = float(line[3]) / r_spacing
            centerline.append([float(line[0]) / x, float(line[1]) / y, float(line[2]) / z, radius])
        else:
            centerline.append([float(line[0]) / x, float(line[1]) / y, float(line[2]) / z])
    f.close()
    return centerline


# compute distances between points
def compute_distance_sequence(vessel):
    dis = list()
    cl_len = len(vessel)
    point1 = vessel[0]
    point2 = vessel[1]
    distance_sum = 0
    for index in range(2, cl_len):
        point_dis = math.sqrt(
            (point2[0] - point1[0]) ** 2 + (point2[1] - point1[1]) ** 2 + (point2[2] - point1[2]) ** 2)
        dis.append(point_dis)
        distance_sum += point_dis
    return dis


# 根据1体素对坐标进行等距线性插值，生成一套新的坐标
def compute_interpolate_point(vessel_centerline, distance):
    interpolate_point_sequence = []
    sum_distance = 0
    start_index = 0
    end_index = 0
    point_len = len(vessel_centerline)
    dis_len = len(distance)
    min_len = min(point_len, dis_len)

    while start_index < min_len and end_index < min_len:
        # 根据1体素弧长寻找起始点和结束点
        while sum_distance < 1 and end_index < min_len:
            sum_distance += distance[end_index]
            end_index += 1
        # 找到了弧长累计为1体素的点，然后求出xyz对应的坐标和半径
        x = round(vessel_centerline[start_index][0] + (
            vessel_centerline[end_index][0] - vessel_centerline[start_index][0]) / sum_distance)
        y = round(vessel_centerline[start_index][1] + (
            vessel_centerline[end_index][1] - vessel_centerline[start_index][1]) / sum_distance)
        z = round(vessel_centerline[start_index][2] + (
            vessel_centerline[end_index][2] - vessel_centerline[start_index][2]) / sum_distance)

        radius = vessel_centerline[start_index][3]
        interpolate_point_sequence.append([x, y, z, radius])
        # 重置相关参数
        start_index = end_index

        sum_distance = 0
    # 返回插值之后的坐标
    return interpolate_point_sequence


# 建立格点空间和采样空间，将格点空间初始化为0，同时计算出每个采样点对应的值。
# 针对每个点周围的3*3 空间进行计算
# 2016.5.2.1 整体采用7*7空间进行计算
# 2016.5.2.2 进一步扩大采样空间 , rgbPoint 阈值进行重新赋值 11*11， 添加计数区间值
def compute_voxel_data(points, offset):

    m_grid = []
    for i in range(512):
        row = []
        for j in range(512):
            col = []
            for k in range(272):
                col.append(0)
            row.append(col)
        m_grid.append(row)

    cpt = 0
    for point in points:

        center_x = point[0]
        center_y = point[1]
        center_z = point[2]
        radius = point[3]

        # 设定针对某个点空间的范围
        x_left_range = max(0, int(center_x - offset))
        x_right_range = min(512, int(center_x + offset))
        y_left_range = max(0, int(center_y - offset))
        y_right_range = min(512, int(center_y + offset))
        z_left_range = max(0, int(center_z - offset))
        z_right_range = min(272, int(center_z + offset))

        if cpt is 0:
            print point
            print x_left_range, x_right_range, y_left_range, y_right_range, z_left_range, z_right_range

        sigma = radius / 2

        for x in xrange(x_left_range, x_right_range):
            for y in xrange(y_left_range, y_right_range):
                for z in xrange(z_left_range, z_right_range):

                    distance = (x - center_x) ** 2 + (y - center_y) ** 2 + (z - center_z) ** 2
                    cal_radius = 2 * sigma ** 2

                    grey_scale_value = math.exp(-(distance / cal_radius)) * 1000

                    # threshold_value = ( 2 * sigma**2 / distance )*100000

                    m_grid[x][y][z] = max(grey_scale_value, m_grid[x][y][z])
        cpt += 1

    return m_grid


def volume_data_process(voxel_data):
    img_data = vtk.vtkImageData()
    img_data.SetDimensions(len(voxel_data), len(voxel_data[0]), len(voxel_data[0][0]))
    # 设定传入的数据类型
    img_data.AllocateScalars(vtk.VTK_UNSIGNED_CHAR, 1)
    dims = img_data.GetDimensions()
    print dims
    start_time = time.time()
    print "start initializing renderer."

    for x in xrange(dims[0]):
        for y in xrange(dims[1]):
            for z in xrange(dims[2]):
                # print "%d, %d, %d okay" %(x, y, z)
                img_data.SetScalarComponentFromFloat(x, y, z, 0, voxel_data[x][y][z])
    print "end initialzing renderer."
    end_time = time.time()
    print end_time - start_time
    return img_data


def CheckAbort(obj, event):
    if obj.GetEventPending() != 0:
        obj.SetAbortRender(1)


# vtk 生成actor
def create_vessel_actor(ref_data):
    # vessel_ref_data = ref_data

    points = vtk.vtkPoints()

    # insert each properties of points into obj.
    for i in xrange(len(ref_data)):
        x = ref_data[i][0]
        y = ref_data[i][1]
        z = ref_data[i][2]
        points.InsertPoint(i, x, y, z)

    poly = vtk.vtkPolyVertex()
    poly.GetPointIds().SetNumberOfIds(len(ref_data))

    cont = 0
    while cont < len(ref_data):
        poly.GetPointIds().SetId(cont, cont)
        cont += 1

    grid = vtk.vtkUnstructuredGrid()
    grid.SetPoints(points)
    grid.InsertNextCell(poly.GetCellType(), poly.GetPointIds())

    mapper = vtk.vtkDataSetMapper()
    if sys.hexversion == 34015984:
        mapper.SetInputData(grid)
    if sys.hexversion == 34015728:
        mapper.SetInput(grid)

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    return actor


if __name__ == '__main__':
    max_x = max_y = max_z = 0

    file_path = "/Users/vincent/Documents/data/dataset03/vessel0/reference.txt"
    x_spacing = 0.363281011581
    y_spacing = 0.363281011581
    z_spacing = 0.40000000596

    vessel = do_parse_vessel_file(file_path, x_spacing, y_spacing, z_spacing)

    # distances = compute_distance_sequence(vessel)

    # interpolate_point_data = compute_interpolate_point(vessel, distances)

    print "start vessel reconstruct procedure"
    rebuild_cube = compute_voxel_data(vessel, 30)

    print "vtk image data generate"
    result_img_volume_data = volume_data_process(rebuild_cube)

    # Create the standard renderer, render window and interactor
    ren = vtk.vtkRenderer()
    renWin = vtk.vtkRenderWindow()
    renWin.AddRenderer(ren)
    iren = vtk.vtkRenderWindowInteractor()
    iren.SetRenderWindow(renWin)

    # 将原坐标和插值之后的坐标中心线显示在空间上
    ref_actor = create_vessel_actor(vessel)
    interp_actor = create_vessel_actor(vessel)
    ref_actor.GetProperty().SetColor(100, 100, 100)
    interp_actor.GetProperty().SetColor(0, 0, 255)
    ren.AddActor(ref_actor)
    ren.AddActor(interp_actor)

    # Create transfer mapping scalar value to opacity
    opacityTransferFunction = vtk.vtkPiecewiseFunction()
    opacityTransferFunction.AddPoint(0, 0.0)
    opacityTransferFunction.AddPoint(50, 0.0)
    opacityTransferFunction.AddPoint(100, 0.8)
    opacityTransferFunction.AddPoint(1200, 0.8)

    # Create transfer mapping scalar value to color
    colorTransferFunction = vtk.vtkColorTransferFunction()
    colorTransferFunction.AddRGBPoint(0, 0.0, 0.0, 0.0)
    colorTransferFunction.AddRGBPoint(50, 0.0, 0.0, 0.0)
    colorTransferFunction.AddRGBPoint(100, 1.0, 0.0, 0.0)
    colorTransferFunction.AddRGBPoint(1200, 1.0, 0.0, 0.0)

    # The property describes how the data will look
    volumeProperty = vtk.vtkVolumeProperty()
    volumeProperty.SetColor(colorTransferFunction)
    volumeProperty.SetScalarOpacity(opacityTransferFunction)
    volumeProperty.ShadeOff()
    volumeProperty.SetInterpolationTypeToLinear()

    # 梯度函数
    # volumeGradientFunction = vtk.vtkGradientTransferFunc

    # The mapper / ray cast function know how to render the data
    compositeFunction = vtk.vtkVolumeRayCastCompositeFunction()
    volumeMapper = vtk.vtkVolumeRayCastMapper()
    volumeMapper.SetVolumeRayCastFunction(compositeFunction)
    volumeMapper.SetInputData(result_img_volume_data)
    volumeMapper.SetBlendModeToMaximumIntensity()

    # The volume holds the mapper and the property and
    # can be used to position/orient the volume
    volume = vtk.vtkVolume()
    volume.SetMapper(volumeMapper)
    volume.SetProperty(volumeProperty)

    ren.AddVolume(volume)
    ren.SetBackground(1, 1, 1)
    renWin.SetSize(600, 600)
    renWin.Render()

    renWin.AddObserver("AbortCheckEvent", CheckAbort)

    iren.Initialize()
    renWin.Render()
    iren.Start()
