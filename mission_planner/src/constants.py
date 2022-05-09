from geometry_msgs.msg import Point
from POI import POI
from utils import *

huldra_smaller_walkway_line = [
    Point(3.433365333333356, -24.15834533333333, 2.4600009999999948),
    Point(0.866699666666662, -24.4916786666667, 2.4600009999999948),
    Point(-1.9056573333333375, -24.45834333333329, 2.4600009999999948),
    Point(-2.0086616666666686, -24.108693333333292, 2.4600009999999948),
    Point(-2.4005176666666586, -24.31470733333333, 2.4600009999999948),
    Point(-2.3904343333333173, -23.950347666666687, 2.4600009999999948),
    Point(-2.8267746666666653, -24.025187000000017, 2.4600009999999948),
    Point(-2.7045896666666636, -23.68177266666669, 2.4600009999999948),
    Point(-3.1426999999999907, -23.61811300000005, 2.4600009999999948),
    Point(-2.920374333333328, -23.329264000000023, 2.4600009999999948),
    Point(-3.3173699999999826, -23.133331, 2.4600009999999948),
    Point(-3.016664999999989, -22.92731699999996, 2.4600009999999948),
    Point(-3.366635333333349, -17.716674666666677, 2.4600009999999948),
    Point(-3.033304666666666, -12.608337333333338, 2.4600009999999948)
]

huldra_small_area_walkway_line = [
    Point(3.16668966666667, 4.666666666666686, 2.4600009999999948),
    Point(0.33335633333332737, 4.333333333333314, 2.4600009999999948),
    Point(-3.133308333333332, 4.866658666666638, 2.4600009999999948),
    Point(-3.766639666666663, 4.433329333333347, 2.4600009999999948),
    Point(-4.966323666666668, 4.214141666666649, 2.4600009999999948),
    Point(-5.532989333333333, 3.1281433333332984, 2.4600009999999948),
    Point(-4.732991333333345, 0.37547833333331937, 2.4600009999999948),
    Point(-5.066324666666674, -1.2911883333333094, 2.4600009999999948),
    Point(-4.736719666666659, -2.918691000000024, 2.4600009999999948),
    Point(-5.070462666666671, -2.9239096666666455, 2.4600009999999948),
    Point(-4.777092333333329, -3.0903729999999996, 2.4600009999999948),
    Point(-5.109542999999988, -3.048146666666696, 2.4600009999999948),
    Point(-4.883194000000003, -3.3022053333333474, 2.4600009999999948),
    Point(-5.186327666666671, -3.1593116666666106, 2.4600009999999948),
    Point(-4.962880333333331, -3.4118553333333352, 2.4600009999999948),
    Point(-5.266013999999998, -3.268961666666655, 2.4600009999999948),
    Point(-5.039664999999999, -3.5230203333333066, 2.4600009999999948),
    Point(-5.372116000000005, -3.480794000000003, 2.4600009999999948),
    Point(-5.078745666666649, -3.6472573333333003, 2.4600009999999948),
    Point(-5.412488666666675, -3.6524760000000356, 2.4600009999999948),
    Point(-5.083669333333333, -3.9331359999999904, 2.4600009999999948),
    Point(-5.417002666666676, -4.256439, 2.4600009999999948),
    Point(-5.082046333333324, -4.590047000000027, 2.4600009999999948),
    Point(-5.39743533333332, -4.70334866666667, 2.4600009999999948),
    Point(-5.057736666666656, -4.722940999999992, 2.4600009999999948),
    Point(-5.32267233333333, -4.928161333333321, 2.4600009999999948),
    Point(-4.997520333333327, -4.838429666666627, 2.4600009999999948),
    Point(-5.229825333333338, -5.078114666666636, 2.4600009999999948),
    Point(-4.714045333333331, -5.600260666666657, 2.4600009999999948),
    Point(-3.9998676666666597, -5.8430380000000355, 2.4600009999999948),
    Point(-3.4535220000000066, -6.401041666666686, 2.4600009999999948),
    Point(-3.6552560000000085, -6.676513666666665, 2.4600009999999948),
    Point(-3.256919999999994, -6.7061970000000315, 2.4600009999999948),
    Point(-3.479245666666671, -6.995046000000002, 2.4600009999999948),
    Point(-3.082250000000002, -7.19097899999997, 2.4600009999999948),
    Point(-3.3829549999999955, -7.396993000000009, 2.4600009999999948)
]

huldra_big_walkway_line = [
    Point(18.66668966666667, 4.666666666666686, 7.059999000000001),
    Point(12.333356333333327, 4.333333333333314, 7.059999000000001),
    Point(2.5168429999999944, 4.666666666666686, 7.059999000000001),
    Point(-0.9664890000000099, 4.333333333333314, 7.059999000000001),
    Point(-4.782987333333324, -0.9664916666666841, 7.059999000000001),
    Point(-5.1163206666666525, -6.933166333333304, 7.059999000000001),
    Point(-3.9846570000000128, -15.733327333333307, 7.059999000000001),
    Point(-4.484657000000013, -16.066660666666678, 7.059999000000001),
    Point(-3.24365233333333, -15.492329666666649, 7.059999000000001),
    Point(-3.365661666666668, -15.947672333333344, 7.059999000000001),
    Point(-4.750678999999991, -13.106008000000031, 7.059999000000001),
    Point(-5.035069666666672, -13.415029000000004, 7.059999000000001),
    Point(-4.574414333333337, -13.806884999999966, 7.059999000000001),
    Point(-4.749392, -14.188660000000027, 7.059999000000001),
    Point(-4.19019066666668, -14.41898633333335, 7.059999000000001),
    Point(-4.238632333333328, -14.836151333333362, 7.059999000000001),
    Point(-3.6356276666666645, -14.882395666666696, 7.059999000000001),
    Point(-3.5527876666666742, -15.294108333333327, 7.059999000000001),
    Point(-2.9650013333333476, -15.15175400000004, 7.059999000000001),
    Point(-2.758989999999997, -15.517720666666662, 7.059999000000001),
    Point(-0.3879876666666746, -15.233327333333307, 7.059999000000001),
    Point(-1.4213233333333477, -15.566660666666678, 7.059999000000001),
    Point(0.8807223333333241, -15.60501099999999, 7.059999000000001),
    Point(0.409317333333334, -15.60501099999999, 7.059999000000001),
    Point(1.0166903333333295, -18.341664666666645, 7.059999000000001),
    Point(0.6833596666666608, -21.08332333333334, 7.059999000000001),
    Point(34.03335833333334, 4.666666666666686, 7.059999000000001),
    Point(29.516690666666662, 4.333333333333314, 7.059999000000001),
    Point(41.88335933333333, 4.666666666666686, 7.059999000000001),
    Point(40.216692666666674, 4.333333333333314, 7.059999000000001),
    Point(35.51668766666667, 6.733317333333332, 7.059999000000001),
    Point(35.18335433333333, 5.866658666666638, 7.059999000000001),
    Point(4.116699666666662, -24.15834533333333, 7.059999000000001),
    Point(2.233368333333331, -24.4916786666667, 7.059999000000001),
    Point(39.216695333333334, -12.608337333333338, 7.059999000000001),
    Point(38.88336466666665, -18.716674666666677, 7.059999000000001),
    Point(33.56669866666665, -22.89166266666666, 7.059999000000001),
    Point(33.233365333333325, -23.35833733333334, 7.059999000000001),
    Point(39.216692666666674, 2.3333333333333144, 7.059999000000001),
    Point(38.88335933333333, -0.3333333333333144, 7.059999000000001),
    Point(39.216692666666674, -4.166666666666686, 7.059999000000001),
    Point(38.88335933333333, -5.333333333333314, 7.059999000000001),
    Point(40.88335933333333, -4.366658666666638, 7.059999000000001),
    Point(40.216692666666674, -5.433329333333347, 7.059999000000001),
    Point(47.01618666666667, -1.5333453333333296, 7.059999000000001),
    Point(42.78285433333333, -1.8666786666667008, 7.059999000000001),
    Point(36.755249000000006, -24.15834533333333, 7.059999000000001),
    Point(31.37764, -24.4916786666667, 7.059999000000001),
    Point(41.01669866666667, -21.558339333333322, 7.059999000000001),
    Point(40.28336633333333, -21.891672666666693, 7.059999000000001),
    Point(19.333364333333336, -24.15834533333333, 7.059999000000001),
    Point(12.666697666666664, -24.4916786666667, 7.059999000000001),
    Point(21.51668766666667, -1.219177000000002, 7.059999000000001),
    Point(21.18335433333334, -6.425842000000046, 7.059999000000001),
    Point(21.20002199999999, -10.96584033333329, 7.059999000000001),
    Point(20.55002300000001, -11.299173666666604, 7.059999000000001),
    Point(20.56669066666666, -12.545817333333332, 7.059999000000001),
    Point(20.23335733333333, -13.679158666666638, 7.059999000000001),
    Point(21.20002199999999, -15.145833333333314, 7.059999000000001),
    Point(20.55002300000001, -15.479166666666686, 7.059999000000001),
    Point(21.51669033333333, -17.992899666666688, 7.059999000000001),
    Point(21.18335966666666, -20.17329933333332, 7.059999000000001),
    Point(21.533010000000004, -22.456705999999997, 7.059999000000001),
    Point(21.232304999999997, -22.662720000000036, 7.059999000000001),
    Point(21.610295666666673, -22.82938666666672, 7.059999000000001),
    Point(21.34996533333333, -23.059713000000045, 7.059999000000001),
    Point(22.422108999999992, -22.92300433333338, 7.059999000000001),
    Point(22.78553266666667, -22.894948333333332, 7.059999000000001),
    Point(22.541516666666666, -22.561615000000018, 7.059999000000001),
    Point(22.882133666666675, -22.492309666666642, 7.059999000000001),
    Point(21.56897233333335, -23.750692000000015, 7.059999000000001),
    Point(21.869677333333343, -23.544677999999976, 7.059999000000001),
    Point(21.491686666666666, -23.37801133333329, 7.059999000000001),
    Point(21.752016666666677, -23.147684999999967, 7.059999000000001),
    Point(22.63603466666666, -23.284393333333355, 7.059999000000001),
    Point(22.272610999999998, -23.312449000000015, 7.059999000000001),
    Point(22.516626999999986, -23.64578233333333, 7.059999000000001),
    Point(22.176010333333338, -23.71508799999998, 7.059999000000001),
    Point(22.89669533333334, -4.784566333333316, 7.059999000000001),
    Point(22.563364666666658, -13.56913266666669, 7.059999000000001)
]
'''
Point(19.333356333333327, 4.666666666666686, 11.460000999999995),
Point(12.66668966666667, 4.333333333333314, 11.460000999999995),
Point(3.16668966666667, 4.666666666666686, 11.460000999999995),
Point(0.33335633333332737, 4.333333333333314, 11.460000999999995),
Point(-3.133308333333332, 4.866658666666638, 11.460000999999995),
Point(-3.766639666666663, 4.433329333333347, 11.460000999999995),
Point(34.60001899999999, 4.666666666666686, 11.460000999999995),
Point(30.300021, 4.333333333333314, 11.460000999999995),
Point(34.26668766666667, 7.933349333333354, 11.460000999999995),
Point(33.93335433333333, 6.466674666666677, 11.460000999999995),
Point(3.433365333333356, -24.15834533333333, 11.460000999999995),
Point(0.866699666666662, -24.4916786666667, 11.460000999999995),
Point(50.42669433333333, -8.150004000000024, 11.460000999999995),
Point(50.093363666666676, -14.974995999999976, 11.460000999999995),
Point(38.25336700000001, -24.15834533333333, 11.460000999999995),
Point(32.126699000000016, -24.4916786666667, 11.460000999999995),
Point(48.63336466666668, -21.1333213333333, 11.460000999999995),
Point(46.50669633333332, -21.46665466666667, 11.460000999999995),
Point(44.04669466666667, -20.825012000000015, 11.460000999999995),
Point(43.71336133333334, -22.325012000000015, 11.460000999999995),
Point(49.83862566666666, -22.2583213333333, 11.460000999999995),
Point(49.50529233333333, -23.59165466666667, 11.460000999999995),
Point(38.54669466666667, -21.75833133333333, 11.460000999999995),
Point(38.21336133333334, -22.791656666666654, 11.460000999999995),
Point(37.663363333333336, -21.058339333333322, 11.460000999999995),
Point(37.44669866666668, -21.391672666666693, 11.460000999999995),
Point(-3.033304666666666, -12.608337333333338, 11.460000999999995),
Point(-3.366635333333349, -17.716674666666677, 11.460000999999995),
Point(-3.999980000000008, -14.0083213333333, 11.460000999999995),
Point(-4.29997800000001, -14.34165466666667, 11.460000999999995),
Point(-3.016664999999989, -22.92731699999996, 11.460000999999995),
Point(-3.3173699999999826, -23.133331, 11.460000999999995),
Point(-2.920374333333328, -23.329264000000023, 11.460000999999995),
Point(-3.1426999999999907, -23.61811300000005, 11.460000999999995),
Point(-2.7045896666666636, -23.68177266666669, 11.460000999999995),
Point(-2.8267746666666653, -24.025187000000017, 11.460000999999995),
Point(-2.3904343333333173, -23.950347666666687, 11.460000999999995),
Point(-2.4005176666666586, -24.31470733333333, 11.460000999999995),
Point(-2.0086616666666686, -24.108693333333292, 11.460000999999995),
Point(-1.9056573333333375, -24.45834333333329, 11.460000999999995),
Point(-4.732991333333345, 0.37547833333331937, 11.460000999999995),
Point(-5.066324666666674, -1.2911883333333094, 11.460000999999995),
Point(-5.229825333333338, -5.078114666666636, 11.460000999999995),
Point(-4.997520333333327, -4.838429666666627, 11.460000999999995),
Point(-5.32267233333333, -4.928161333333321, 11.460000999999995),
Point(-5.057736666666656, -4.722940999999992, 11.460000999999995),
Point(-5.39743533333332, -4.70334866666667, 11.460000999999995),
Point(-5.082046333333324, -4.590047000000027, 11.460000999999995),
Point(-5.083669333333333, -3.9331359999999904, 11.460000999999995),
Point(-5.417002666666676, -4.256439, 11.460000999999995),
Point(-4.966323666666668, 4.214141666666649, 11.460000999999995),
Point(-5.532989333333333, 3.1281433333332984, 11.460000999999995),
Point(-3.4535220000000066, -6.401041666666686, 11.460000999999995),
Point(-3.6552560000000085, -6.676513666666665, 11.460000999999995),
Point(-3.256919999999994, -6.7061970000000315, 11.460000999999995),
Point(-3.479245666666671, -6.995046000000002, 11.460000999999995),
Point(-3.082250000000002, -7.19097899999997, 11.460000999999995),
Point(-3.3829549999999955, -7.396993000000009, 11.460000999999995),
Point(-5.412488666666675, -3.6524760000000356, 11.460000999999995),
Point(-5.078745666666649, -3.6472573333333003, 11.460000999999995),
Point(-5.372116000000005, -3.480794000000003, 11.460000999999995),
Point(-5.039664999999999, -3.5230203333333066, 11.460000999999995),
Point(-5.266013999999998, -3.268961666666655, 11.460000999999995),
Point(-4.962880333333331, -3.4118553333333352, 11.460000999999995),
Point(-4.736719666666659, -2.918691000000024, 11.460000999999995),
Point(-5.070462666666671, -2.9239096666666455, 11.460000999999995),
Point(-4.777092333333329, -3.0903729999999996, 11.460000999999995),
Point(-5.109542999999988, -3.048146666666696, 11.460000999999995),
Point(-4.883194000000003, -3.3022053333333474, 11.460000999999995),
Point(-5.186327666666671, -3.1593116666666106, 11.460000999999995),
Point(-4.714045333333331, -5.600260666666657, 11.460000999999995),
Point(-3.9998676666666597, -5.8430380000000355, 11.460000999999995),
Point(40.23335733333333, 2.8916626666666616, 11.460000999999995),
Point(39.56669066666667, 0.7833253333333232, 11.460000999999995),
Point(43.303354666666664, -1.6583453333333296, 11.460000999999995),
Point(41.10168933333334, -1.9916786666667008, 11.460000999999995),
Point(48.37168866666667, -1.6583453333333296, 11.460000999999995),
Point(46.938354333333336, -1.9916786666667008, 11.460000999999995),
Point(43.86669166666667, -2.508321333333299, 11.460000999999995),
Point(42.93335733333333, -2.8416546666666704, 11.460000999999995),
Point(19.333364333333336, -24.15834533333333, 11.460000999999995),
Point(12.666697666666664, -24.4916786666667, 11.460000999999995),
Point(22.216695333333334, -4.734588999999971, 11.460000999999995),
Point(21.88336466666665, -13.469146999999964, 11.460000999999995),
Point(22.368975333333324, -23.773885000000007, 11.469999000000001),
Point(22.669680333333346, -23.56787099999997, 11.469999000000001),
Point(22.29168966666667, -23.401204333333283, 11.469999000000001),
Point(22.552019666666652, -23.17087799999996, 11.469999000000001),
Point(22.428665333333342, -22.928079999999966, 11.469999000000001),
Point(22.17332466666666, -23.15248600000001, 11.469999000000001),
Point(22.233007, -22.306712000000005, 11.469999000000001),
Point(21.932301999999993, -22.512725999999986, 11.469999000000001),
Point(22.31029266666667, -22.679392666666672, 11.469999000000001),
Point(22.04996233333334, -22.909719000000052, 11.469999000000001),
Point(23.716692666666674, -3.099334666666664, 11.460000999999995),
Point(23.38335933333333, -6.824676333333343, 11.460000999999995),
Point(23.716695333333334, -16.696655333333354, 11.460000999999995),
Point(23.38336466666665, -18.023315666666633, 11.460000999999995),
Point(24.25668566666667, -12.379160666666678, 11.460000999999995),
Point(23.92335233333334, -13.545827333333307, 11.460000999999995),
Point(24.20669066666666, 3.200012000000015, 11.460000999999995),
Point(23.87335733333333, 2.200012000000015, 11.460000999999995),
Point(24.20669766666667, -21.224995999999976, 11.460000999999995),
Point(23.873364333333328, -22.525004000000024, 11.460000999999995),
Point(23.852694333333332, -10.605763999999965, 11.460000999999995),
Point(23.787351333333334, -11.159250999999983, 11.460000999999995),
Point(24.076026999999996, -14.932434, 11.460000999999995),
Point(23.564018333333323, -15.15258799999998, 11.460000999999995),
Point(24.042691666666656, 1.0081480000000056, 11.460000999999995),
Point(23.547350666666674, 0.8168640000000096, 11.460000999999995),
Point(23.836033999999998, -19.37520333333333, 11.460000999999995),
Point(23.754023000000004, -19.8998206666667, 11.460000999999995),
Point(25.683357333333333, -11.832845333333296, 11.460000999999995),
Point(24.866691666666668, -12.166178666666667, 11.460000999999995),
Point(27.632863333333333, -10.430948999999998, 11.460000999999995),
Point(27.067179333333343, -11.468038000000035, 11.460000999999995),
Point(30.966689666666667, -9.732839333333288, 11.460000999999995),
Point(29.58335833333335, -10.06617266666666, 11.460000999999995),
Point(20.833356333333327, 0.16666666666668561, 17.459999000000003),
Point(15.66668966666667, -0.16666666666668561, 17.459999000000003),
Point(8.733355333333336, 2.100016000000039, 17.459999000000003),
Point(8.466687666666658, 1.3000079999999912, 17.459999000000003),
Point(28.15004766666665, 0.16666666666668561, 17.459999000000003),
Point(27.075035333333332, -0.16666666666668561, 17.459999000000003),
Point(32.509129999999985, -0.8333333333333144, 17.459999000000003),
Point(30.367091000000002, -1.1666666666666856, 17.459999000000003),
Point(32.84244299999999, 4.266672666666693, 17.459999000000003),
Point(31.033777999999998, 3.933339333333322, 17.459999000000003),
Point(8.035021666666651, 1.1666666666666856, 17.459999000000003),
Point(4.570020333333332, 0.8333333333333144, 17.459999000000003),
Point(11.16668966666667, -2.3333333333333144, 17.459999000000003),
Point(10.833356333333327, -5.166666666666686, 17.459999000000003),
Point(7.034688333333335, -16.226664000000028, 17.459999000000003),
Point(6.701357666666652, -18.513326000000006, 17.459999000000003),
Point(9.397761000000003, -10.013132666666706, 17.459999000000003),
Point(8.134175666666678, -12.57091266666663, 17.459999000000003),
Point(6.745351333333318, -8.620554666666692, 17.459999000000003),
Point(6.90658066666667, -9.063527333333298, 17.459999000000003),
Point(29.558393333333342, 7.166666666666686, 17.459999000000003),
Point(29.39172666666667, 6.833333333333314, 17.459999000000003),
Point(39.401166, -0.8333333333333144, 19.169998),
Point(38.101168, -1.1666666666666856, 19.169998),
Point(39.967829666666674, -1.5633443333333616, 19.169998),
Point(39.63449633333333, -1.6366676666666535, 19.169998),
Point(36.802329666666665, -0.8333333333333144, 19.17455566666667),
Point(36.80643433333334, -1.1666666666666856, 19.16930133333333),
Point(36.06274633333334, -0.8333333333333144, 18.605192000000002),
Point(35.32726766666667, -1.1666666666666856, 18.030574),
Point(41.010086, -0.3213706666666667, 19.169998),
Point(40.84509266666667, -0.9106243333333168, 19.169998),
Point(39.966692666666674, -3.566670666666653, 18.109997),
Point(39.63335933333333, -4.133341333333362, 18.109997),
Point(39.966692666666674, -2.1477256666667017, 18.827936666666666),
Point(39.63335933333333, -2.5849713333333852, 18.467492333333333),
Point(39.63335933333333, -1.7019960000000083, 19.178094333333334),
Point(39.966692666666674, -1.7062379999999848, 19.183237666666663),
Point(40.243436, -4.787516333333315, 18.109997),
Point(40.416613, -5.51247166666667, 18.109997),
Point(44.466692666666674, -10.839996333333318, 18.109997),
Point(44.13335933333333, -14.2799986666667, 18.109997),
Point(43.916694333333325, -18.12000500000005, 18.109997),
Point(42.80836466666666, -18.520009000000016, 18.109997),
Point(41.30002333333333, -18.859995333333302, 18.109997),
Point(40.90002700000001, -20.009999666666715, 18.109997),
Point(44.75836433333335, -19.453338333333306, 18.459999000000003),
Point(44.49169666666667, -19.986663666666686, 18.459999000000003),
Point(55.15716533333333, -9.333333333333314, 19.371997999999998),
Point(53.54749666666667, -9.666666666666686, 19.371997999999998),
Point(51.95798466666666, -9.333333333333314, 19.365679666666665),
Point(51.96208933333334, -9.666666666666686, 19.360425333333332),
Point(51.41540766666667, -9.333333333333314, 18.950231333333335),
Point(50.876935333333336, -9.666666666666686, 18.529528666666664),
Point(48.53565933333334, -9.333333333333314, 18.109997),
Point(46.66899066666667, -9.666666666666686, 18.109997),
Point(43.23365766666666, -6.200001999999984, 18.109997),
Point(41.666992333333326, -6.799998000000016, 18.109997),
Point(59.939635, -7.443318333333309, 19.371997999999998),
Point(59.539636, -7.776651666666623, 19.371997999999998),
Point(57.565641, -9.042226000000028, 19.371997999999998),
Point(58.35363266666666, -8.07776899999999, 19.371997999999998),
Point(32.591695333333334, -15.666666666666686, 17.459999000000003),
Point(32.25836466666665, -19.333333333333314, 17.459999000000003),
Point(31.71300766666667, -3.692901666666671, 17.459999000000003),
Point(31.37967433333334, -6.785797333333335, 17.459999000000003),
Point(32.25425466666667, -11.947855666666669, 17.459999000000003),
Point(32.57938133333333, -11.843567000000007, 17.459999000000003),
Point(32.209716666666665, -11.692240666666692, 17.459999000000003),
Point(32.47449233333333, -11.441721666666695, 17.459999000000003),
Point(32.05173466666666, -11.310313000000008, 17.459999000000003),
Point(32.22613766666666, -10.990234333333376, 17.459999000000003),
Point(31.79988866666666, -10.97765099999998, 17.459999000000003),
Point(32.00162266666666, -10.702179000000001, 17.459999000000003),
Point(31.60328700000001, -10.672495666666634, 17.459999000000003),
Point(31.82561266666667, -10.383647000000053, 17.459999000000003),
Point(31.428617000000003, -10.187713999999971, 17.459999000000003),
Point(31.72932200000001, -9.98169999999999, 17.459999000000003),
Point(28.89172666666667, 0.9333193333333156, 17.459999000000003),
Point(28.558393333333342, -0.333343333333346, 17.459999000000003),
Point(28.89172666666667, 6.133341333333362, 17.459999000000003),
Point(28.558393333333342, 4.766682666666668, 17.459999000000003),
Point(28.62506599999999, 3.233327333333307, 17.459999000000003),
Point(29.025064999999998, 3.066660666666678, 17.459999000000003),
Point(29.091730666666663, 2.8333233333333396, 17.459999000000003),
Point(28.758397333333335, 2.76665266666663, 17.459999000000003),
Point(28.69173166666667, 2.5333453333333296, 17.459999000000003),
Point(28.958399333333332, 2.366678666666701, 17.459999000000003),
Point(17.933364999999995, -23.03334533333333, 17.459999000000003),
Point(14.066695999999993, -23.3666786666667, 17.459999000000003),
Point(24.266698666666684, -22.333333333333314, 17.459999000000003),
Point(22.533366333333333, -22.666666666666686, 17.459999000000003),
Point(9.589360333333346, -21.1333213333333, 17.459999000000003),
Point(7.978693666666672, -21.46665466666667, 17.459999000000003),
Point(4.712033333333309, -22.1333213333333, 17.459999000000003),
Point(2.056032666666667, -22.46665466666667, 17.459999000000003),
Point(8.966700666666668, -22.00001000000003, 17.459999000000003),
Point(8.63336733333334, -22.200001999999984, 17.459999000000003),
Point(10.866693666666677, -22.099995999999976, 17.459999000000003),
Point(10.533360333333349, -22.400004000000024, 17.459999000000003),
Point(0.06669333333331906, -17.23333733333334, 17.459999000000003),
Point(-0.2666373333333354, -19.51666266666666, 17.459999000000003),
Point(29.070696666666663, -22.333333333333314, 17.459999000000003),
Point(27.166364333333334, -22.666666666666686, 17.459999000000003),
Point(30.641695666666678, -21.30000799999999, 17.459999000000003),
Point(30.308362333333335, -22.150004000000024, 17.459999000000003),
Point(35.941693666666666, -20.78334533333333, 17.459999000000003),
Point(32.95836133333334, -21.1166786666667, 17.459999000000003),
Point(40.383362000000005, -20.41667666666666, 18.109997),
Point(40.266693000000004, -20.783335333333355, 18.109997),
Point(39.120031, -20.41667666666666, 17.459999000000003),
Point(39.020030000000006, -20.783335333333355, 17.459999000000003),
Point(39.821298999999996, -20.783335333333355, 17.893128666666662),
Point(39.507290999999995, -20.41667666666666, 17.67325833333333),
Point(40.14295166666666, -20.41667666666666, 18.102076333333336),
Point(40.13912933333333, -20.783335333333355, 18.107537666666666),
Point(38.59170066666667, -20.933329333333347, 17.459999000000003),
Point(38.25836733333334, -21.816670666666653, 17.459999000000003),
Point(-1.110473666666664, -16.03334533333333, 17.459999000000003),
Point(-2.621475333333322, -16.3666786666667, 17.459999000000003),
Point(4.044947333333326, -14.28334533333333, 17.459999000000003),
Point(1.7222796666666653, -14.6166786666667, 17.459999000000003),
Point(9.250795666666662, -8.510884333333308, 17.459999000000003),
Point(7.917462333333333, -8.84421766666668, 17.459999000000003),
Point(-4.2986476666666675, -14.183339333333322, 17.459999000000003),
Point(-4.465314333333325, -14.516672666666693, 17.459999000000003)
'''

huldra_smaller_points_of_interest = [
    POI('20-2000VF', obj_to_gazebo_point([-117.014, 30.016, 304.500]), orientation_from_euler(0, 0, -pi/2)), # "x": 304500, "y": 117014, "z": 30016
    POI('20-2007VF', obj_to_gazebo_point([-115.739, 31.149, 304.950]), orientation_from_euler(0, 0, -pi)),
    POI('20-2003VF', obj_to_gazebo_point([-114.401, 30.099, 307.900]), orientation_from_euler(0, -pi/4, -pi)), # not successful (angle weighted too little)
    POI('20-2006PL', obj_to_gazebo_point([-112.550, 30.238, 310.292]), orientation_from_euler(0, 0, -pi)),
    POI('20-2001PL', obj_to_gazebo_point([-111.588, 30.156, 310.117]), orientation_from_euler(0, 0, -pi)),
    POI('20-2031PL', obj_to_gazebo_point([-112.772, 29.722, 309.125]), orientation_from_euler(0, -pi/2, 0)),
    POI('20-2001WI', obj_to_gazebo_point([-113.725, 29.311, 301.05]), orientation_from_euler(0, 0, -pi/2)), # not successful (3/4 impossible to see, the last one due to mission outer edge of walkway)
    POI('20-2002WI', obj_to_gazebo_point([-113.725, 29.311, 305.1]), orientation_from_euler(0, 0, pi/2)), # not successful
    POI('20-2003WI', obj_to_gazebo_point([-111.975, 29.602, 304.68]), orientation_from_euler(0, -pi/4, pi/2)), # not successful
    POI('20-2004WI', obj_to_gazebo_point([-113.725, 29.581, 302.844]), orientation_from_euler(0, -pi/4, -pi)), # not successful
    POI('20-2005WI', obj_to_gazebo_point([-113.425, 29.445, 305.985]), orientation_from_euler(0, -pi/4, -pi)),
    POI('20-2006WI', obj_to_gazebo_point([-113.703, 29.456, 306.387]), orientation_from_euler(0, -pi/4, -pi)),
    POI('20-2001VF', obj_to_gazebo_point([-115.739, 30.729, 304.041]), orientation_from_euler(0, -pi/2, 0)),
    POI('20-2008VF', obj_to_gazebo_point([-115.739, 31.296, 305.2]), orientation_from_euler(0, 0, -pi)),
    POI('43-4507VF', obj_to_gazebo_point([-118.245, 31.027, 304.477]), orientation_from_euler(0, 0, -pi))
]