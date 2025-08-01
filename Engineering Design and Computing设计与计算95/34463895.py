from Offset_reader import *
import math
from demo_heeled_data import *




def rectangle_params(p1, p2):
    y1, z1 = p1
    y2, z2 = p2

    if y1 == y2 or z1 == z2:
        return "This is not a rectangle!"

    A = abs((y2 - y1) * (z2 - z1))

    H_L = (y1 + y2) / 2
    V_L = (z1 + z2) / 2

    return A, H_L, V_L


def triangle_params(p1, p2):
    global H_L
    y1, z1 = p1
    y2, z2 = p2

    if y1 == y2 or z1 == z2:
        return "This is not a triangle!"



    A = abs((y2 - y1) * (z2 - z1)) / 2
    d_h = abs(y2 - y1)
    d_v = abs(z2 - z1)

    if y2 > y1:
        H_L = y1 + d_h / 3
        V_L = z1 + d_v * 2 / 3
    if y1 > y2:
        H_L = y2 + d_h / 3
        V_L = z1 + d_v / 3

    return A, H_L, V_L


def trapezium_params(p1, p2):
    global H_r
    y1, z1 = p1
    y2, z2 = p2
    if (y1 == 0 and y2 == 0) or (z1 == z2):
        A=0
        H_L=0
        V_L=0
        return A, H_L, V_L
    elif (y1 == 0 and y2 != 0) or (y1 != 0 and y2 == 0):
        A, H_L, V_L=triangle_params(p1, p2)
        return A, H_L, V_L
    elif y1 == y2 and y1 != 0:
        y0 = 0
        z0 = z1
        A, H_L, V_L = rectangle_params((y0, z0), (y2, z2))
        return A, H_L, V_L
    else:
        if y2 > y1:
            y0 = y1
            z0 = z2
            A_r, H_r, V_r = rectangle_params((0, z1), (y0, z0))

        if y1 > y2:
            y0 = y2
            z0 = z1
            A_r, H_r, V_r = rectangle_params((0, z2), (y0, z0))

        A_t, H_t, V_t = triangle_params(p1, p2)
        A = A_r + A_t
        H_L = (H_r * A_r + H_t * A_t) / A
        V_L = (V_r * A_r + V_t * A_t) / A

        return A, H_L, V_L



def cumul_list(alist):
    cumul_sum = []
    total = 0

    for num in alist:
        total += num
        cumul_sum.append(total)

    return cumul_sum



def bonjean(data):
    b_areas = []
    b_vertical_moment = []
    b_horizontal_moment = []
    alist = [0]
    vlist = [0]
    hlist = [0]

    for i in range(len(data) - 1):
        p1 = data[i]
        p2 = data[i + 1]
        A, H_L, V_L = trapezium_params(p1, p2)
        alist.append(A)
        vlist.append(V_L * A)
        hlist.append(H_L * A)

    for i in range(len(data)):
        b_areas.append((cumul_list(alist)[i], data[i][1]))

        b_vertical_moment.append((cumul_list(vlist)[i], data[i][1]))

        b_horizontal_moment.append((cumul_list(hlist)[i], data[i][1]))

    return b_areas, b_horizontal_moment, b_vertical_moment




def create_bonjeans(filename):
    data = read_offs(filename)
    all_ba=[]
    all_bh=[]
    all_bv=[]
    for i in range(len(data)):
        data_stn = [(pnt[1], pnt[2]) for pnt in data[i]]
        ba, bh, bv = bonjean(data_stn)
        all_ba.append(ba)
        all_bh.append(bh)
        all_bv.append(bv)
    return all_ba,all_bh,all_bv





def interpolate_1D(data, targ):

    sort_data = sorted(data, key=lambda tup: tup[0])


    x_zhi = [tup[0] for tup in sort_data]
    y_zhi = [tup[1] for tup in sort_data]


    if targ < y_zhi[0]:
        return 0
    elif targ > y_zhi[-1]:
        return x_zhi[-1]


    for i in range(len(x_zhi) - 1):
        if y_zhi[i] <= targ <= y_zhi[i + 1]:
            x_interp = x_zhi[i] + (targ - y_zhi[i]) * (x_zhi[i + 1] - x_zhi[i]) / (y_zhi[i + 1] - y_zhi[i])
            return x_interp




def trap_area(xdata, ydata):
    if len(xdata) != len(ydata) or len(xdata) < 2:
        print("input error")

    area = 0.0
    for i in range(1, len(xdata)):
        width = xdata[i] - xdata[i - 1]

        height_avg = (ydata[i] + ydata[i - 1]) / 2.0

        area += width * height_avg

    return area




def hydrostatics(wl, filename):

    hydro_dict = {}
    data = read_offs(filename)
    stn_pos = [stn[0][0] for stn in data]
    ba, bh, bv = create_bonjeans(filename)
    sec_area = [interpolate_1D(ba[i], wl) for i in range(len(ba))]

    Ax_integrate = trap_area(stn_pos, sec_area)
    volume_displacement = 2 * Ax_integrate

    x_times_Ax = [x*y for x,y in zip(stn_pos,sec_area)]
    x_Ax_integrate = trap_area(stn_pos, x_times_Ax)
    LCB = x_Ax_integrate / Ax_integrate

    m_z = [interpolate_1D(bv[i], wl) for i in range(len(bv))]
    Mz_integrate = trap_area(stn_pos, m_z)
    VCB = Mz_integrate / Ax_integrate

    yz_pairs = []
    y_position = []
    for dataset in data:
        for point in dataset:
            yz_pairs.append((point[1], point[2]))
        a = interpolate_1D(yz_pairs, wl)
        yz_pairs=[]
        y_position.append(a)

    y_integrate=trap_area(stn_pos, y_position)
    waterplane_area = 2*y_integrate

    x_time_y = [x * y for x, y in zip(stn_pos, y_position)]
    x_y_integrate = trap_area(stn_pos, x_time_y)
    LCF = x_y_integrate / y_integrate

    x2_time_y = [x ** 2 * y for x, y in zip(stn_pos, y_position)]
    J_L = 2 * trap_area(stn_pos, x2_time_y) - waterplane_area * LCF ** 2
    BML = J_L / volume_displacement

    y3 = [y ** 3 for y in y_position]
    J_T = 2 * trap_area(stn_pos, y3) / 3
    BMT = J_T / volume_displacement

    L = data[-1][0][0] - data[0][0][0]
    max_y = 0
    points_at_x_zero = []
    for dataset in data:
        for x, y, z in dataset:
            if x == 0 and y > max_y:
                max_y = y
            if x == 0:
                points_at_x_zero.append((y, z))
    B = 2 * max_y
    T = wl
    A_m = 2*max(sec_area)
    C_B = volume_displacement / L / B / T
    C_P = volume_displacement / L / A_m
    C_M = C_B / C_P
    C_W = waterplane_area / B / L
    MCT = volume_displacement * BML / 100 / L


    hydro_dict = {
        'Volume': volume_displacement,
        'LCB': LCB,
        'VCB': VCB,
        'Aw': waterplane_area,
        'LCF': LCF,
        'BML': BML,
        'BMT': BMT,
        'Cb': C_B,
        'Cp': C_P,
        'Cm': C_M,
        'Cw': C_W,
        'MCTC': MCT
    }

    return hydro_dict



def intersection(p1, p2, ang, point):
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = point

    if x2 - x1 == 0:
        x_intersect = x1
        slope_2 = math.tan(math.radians(ang))
        y_intersect = slope_2 * (x_intersect - x3) + y3
        return x_intersect, y_intersect

    slope_1 = (y2 - y1) / (x2 - x1)
    slope_2 = math.tan(math.radians(ang))


    if slope_1 == slope_2:
        return "Lines are parallel and will never intersect"


    x_intersect = (slope_1 * x1 - slope_2 * x3 + y3 - y1) / (slope_1 - slope_2)
    y_intersect = slope_1 * (x_intersect - x1) + y1
    return x_intersect, y_intersect


def point_of_int(data, ang, point):
    for i in range(len(data) - 1):
        p1 = data[i]
        p2 = data[i + 1]
        intersect = intersection(p1, p2, ang, point)

        if intersect == "Lines will never intersect":
            continue

        x_intersect, y_intersect = intersect
        if min(p1[0], p2[0]) <= x_intersect <= max(p1[0], p2[0]) and min(p1[1], p2[1]) <= y_intersect <= max(p1[1], p2[1]):
            return x_intersect, y_intersect

    return None



def GZ_calc(filename, ang, point=(0,6),KG=7):

    areas = heeled_sec_areas(filename, ang, point)
    vmmts = heeled_sec_vmmts(filename, ang, point)
    hmmts = heeled_sec_hmmts(filename, ang, point)

    data = read_offs(filename)
    stn_pos = [stn[0][0] for stn in data]


    heel_sec_area = [dian[1] for dian in areas]
    heel_sec_vmmts = [dian[1] for dian in vmmts]
    heel_sec_hmmts = [dian[1] for dian in hmmts]

    heel_volume_displacement=trap_area(stn_pos, heel_sec_area)

    mz_integrate=trap_area(stn_pos,heel_sec_vmmts)
    VCB=mz_integrate/heel_volume_displacement


    my_integrate=trap_area(stn_pos,heel_sec_hmmts)
    HCB=my_integrate/heel_volume_displacement


    B=(HCB,VCB)


    G=(0,KG)


    M=intersection(G,point,ang-90,B)
    _,KM=M


    GM=KM-KG
    GZ=GM*math.sin(math.radians(ang))
    return GZ




def extract_shear_parameters(xdata, shearaft=0.375, shearfwd=0.7, midship=9.0):
    x_aft=xdata[0]
    x_fwd=xdata[-1]
    y_aft=shearaft+midship-0.076
    y_fwd=shearfwd+midship-0.076
    c=midship-0.076
    a_fwd=(y_fwd-c)/x_fwd**2
    a_aft = (y_aft - c) / x_aft ** 2
    aft_params=(a_aft,c)
    fwd_params=(a_fwd,c)
    return aft_params, fwd_params




def extract_damaged_waterlines(x, xdata):
    aft_params, fwd_params = extract_shear_parameters(xdata)
    waterlines = []
    if x>=0:
        z0=fwd_params[0]*x**2+fwd_params[1]
        slope=2*fwd_params[0]*x
    else:
        z0=aft_params[0]*x**2+aft_params[1]
        slope = 2 * aft_params[0] * x
    for i in xdata:
        z=slope*(i-x)+z0
        waterlines.append(z)
    return waterlines


def damaged_hydrostatics(filename, x):
    data = read_offs(filename)
    stn_pos = [stn[0][0] for stn in data]
    ba, bh, bv = create_bonjeans(filename)
    wl=extract_damaged_waterlines(x,stn_pos)
    sec_area = [interpolate_1D(ba[i], wl[i]) for i in range(len(ba))]
    Ax_integrate = trap_area(stn_pos, sec_area)
    volume = 2 * Ax_integrate


    x_times_Ax = [x*y for x,y in zip(stn_pos,sec_area)]
    x_Ax_integrate = trap_area(stn_pos, x_times_Ax)
    LCB = x_Ax_integrate / Ax_integrate

    return volume,LCB



def bonjean_volume(filename, x):
    data = read_offs(filename)
    stn_pos = [stn[0][0] for stn in data]
    x_poi= [stn[0][0]+data[-1][0][0] for stn in data]
    ba, bh, bv = create_bonjeans(filename)
    wl=extract_damaged_waterlines(x,stn_pos)
    sec_area = [interpolate_1D(ba[i], wl[i]) for i in range(len(ba))]
    b_volume_sum=[]
    for k in range(len(sec_area)):
        if k==0:
            Ax_integrate=0
            volume = 2 * Ax_integrate
            b_volume_sum.append(volume)
        if k!=0:
            Ax_integrate = trap_area(x_poi[0:k+1], sec_area[0:k+1])
            volume = 2 * Ax_integrate
            b_volume_sum.append(volume)
    return x_poi, b_volume_sum

def bonjean_moment(filename, x):
    data = read_offs(filename)
    stn_pos = [stn[0][0] for stn in data]
    x_poi= [stn[0][0]+data[-1][0][0] for stn in data]
    ba, bh, bv = create_bonjeans(filename)
    wl = extract_damaged_waterlines(x, stn_pos)
    sec_area = [interpolate_1D(ba[i], wl[i]) for i in range(len(ba))]
    x_times_Ax = [x * y for x, y in zip(stn_pos, sec_area)]
    moment_list=[]

    for i in range(1,len(sec_area)+1):
        Ax_integrate_all = trap_area(stn_pos[0:i], x_times_Ax[0:i])
        moment = 2 * Ax_integrate_all
        moment_list.append(moment)
    return x_poi, moment_list


