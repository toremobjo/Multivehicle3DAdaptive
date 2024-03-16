
import rospy
import numpy as np
from auv_handler import AuvHandler
from pykrige.ok import OrdinaryKriging
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator
import os

import scipy.spatial

pred = np.array([-8.95661056e+03,-2.56981624e+03,-4.46434609e+02,-2.50881686e+01,1.35459026e+01,-9.87279688e+01,-1.60396446e+03,-7.06354107e+03,1.90621277e+03, 3.89771332e+02,-8.23698343e-01,-1.00114349e+01,7.07806626e+00 ,1.52981812e+02, 8.17008861e+02, 2.74359833e+03,-1.80107726e+03,-5.36589848e+02,-1.02686525e+02,-8.46134903e+00, 2.31093598e+00,-5.73595972e+00,-2.67282723e+02,-1.27323530e+03,-1.99047862e+03,-6.90522174e+02,-1.71853918e+02,-1.69106013e+01,1.09511849e+01,7.89021578e+01 , 6.02247965e+00,-6.25251098e+02])

k_bb = np.array([[1.60025000e-01 ,2.08013631e-02 ,1.15174156e-03 ,1.62818095e-04
 ,6.66521586e-06 ,6.27434551e-07 ,5.08589432e-08 ,7.28766123e-09
 ,2.55466791e-09 ,1.13538016e-08 ,4.83123709e-08 ,1.23693584e-07
 ,2.28744752e-07 ,3.42109464e-07 ,1.79267289e-07 ,1.39433490e-07]
 ,[2.08013631e-02 ,1.60025000e-01 ,8.60707117e-03 ,1.23592603e-03
 ,5.01583263e-05 ,4.72552133e-06 ,3.83396489e-07 ,5.51757919e-08
 ,1.94928115e-08 ,8.73304856e-08 ,3.58943486e-07 ,8.41710108e-07
 ,1.31922135e-06 ,1.64253363e-06 ,5.83219780e-07 ,3.58352044e-07]
 ,[1.15174156e-03 ,8.60707117e-03 ,1.60025000e-01 ,2.23984010e-02
 ,9.25918071e-04 ,8.71612647e-05 ,7.06526590e-06 ,1.01186508e-06
 ,3.52391941e-07 ,1.49110073e-06 ,4.84935002e-06 ,8.25463344e-06
 ,8.07089232e-06 ,6.44505446e-06 ,1.09297135e-06 ,4.53545300e-07]
 ,[1.62818095e-04 ,1.23592603e-03 ,2.23984010e-02 ,1.60025000e-01
 ,6.46149941e-03 ,6.10458833e-04 ,4.95862863e-05 ,7.14272250e-06
 ,2.50842629e-06 ,1.06027097e-05 ,3.05658228e-05 ,4.11843994e-05
 ,2.80282091e-05 ,1.60554012e-05 ,1.69329372e-06 ,5.49823838e-07]
 ,[6.66521586e-06 ,5.01583263e-05 ,9.25918071e-04 ,6.46149941e-03
 ,1.60025000e-01 ,1.50615716e-02 ,1.22087924e-03 ,1.74506600e-04
 ,5.97631495e-05 ,2.20800498e-04 ,3.35308567e-04 ,2.01127979e-04
 ,5.62375517e-05 ,1.70656197e-05 ,9.22409479e-07 ,2.15992686e-07]
 ,[6.27434551e-07 ,4.72552133e-06 ,8.71612647e-05 ,6.10458833e-04
 ,1.50615716e-02 ,1.60025000e-01 ,1.29690460e-02 ,1.84894827e-03
 ,6.21840239e-04 ,1.93432926e-03 ,1.23849914e-03 ,3.31939998e-04
 ,4.98943274e-05 ,1.04493639e-05 ,4.18410356e-07 ,8.29470309e-08]
 ,[5.08589432e-08 ,3.83396489e-07 ,7.06526590e-06 ,4.95862863e-05
 ,1.22087924e-03 ,1.29690460e-02 ,1.60025000e-01 ,2.26419345e-02
 ,7.28509813e-03 ,1.30505741e-02 ,1.74935783e-03 ,2.21322770e-04
 ,2.18710971e-05 ,3.59392793e-06 ,1.23926038e-07 ,2.19426161e-08]
 ,[7.28766123e-09 ,5.51757919e-08 ,1.01186508e-06 ,7.14272250e-06
 ,1.74506600e-04 ,1.84894827e-03 ,2.26419345e-02 ,1.60025000e-01
 ,4.82941867e-02 ,2.26600556e-02 ,1.12620385e-03 ,1.08439573e-04
 ,9.30997631e-06 ,1.38386202e-06 ,4.62476670e-08 ,7.73993834e-09]
 ,[2.55466791e-09 ,1.94928115e-08 ,3.52391941e-07 ,2.50842629e-06
 ,5.97631495e-05 ,6.21840239e-04 ,7.28509813e-03 ,4.82941867e-02
 ,1.60025000e-01 ,2.08222130e-02 ,9.65663391e-04 ,8.88972670e-05
 ,7.43556341e-06 ,1.06680713e-06 ,3.60909075e-08 ,5.88478995e-09]
 ,[1.13538016e-08 ,8.73304856e-08 ,1.49110073e-06 ,1.06027097e-05
 ,2.20800498e-04 ,1.93432926e-03 ,1.30505741e-02 ,2.26600556e-02
 ,2.08222130e-02 ,1.60025000e-01 ,7.22659993e-03 ,6.68294715e-04
 ,5.60376608e-05 ,8.09129593e-06 ,2.72335799e-07 ,4.46322182e-08]
 ,[4.83123709e-08 ,3.58943486e-07 ,4.84935002e-06 ,3.05658228e-05
 ,3.35308567e-04 ,1.23849914e-03 ,1.74935783e-03 ,1.12620385e-03
 ,9.65663391e-04 ,7.22659993e-03 ,1.60025000e-01 ,1.47293170e-02
 ,1.23196691e-03 ,1.76370103e-04 ,5.97985797e-06 ,9.73782845e-07]
 ,[1.23693584e-07 ,8.41710108e-07 ,8.25463344e-06 ,4.11843994e-05
 ,2.01127979e-04 ,3.31939998e-04 ,2.21322770e-04 ,1.08439573e-04
 ,8.88972670e-05 ,6.68294715e-04 ,1.47293170e-02 ,1.60025000e-01
 ,1.33818940e-02 ,1.91050586e-03 ,6.49571254e-05 ,1.05663376e-05]
 ,[2.28744752e-07 ,1.31922135e-06 ,8.07089232e-06 ,2.80282091e-05
 ,5.62375517e-05 ,4.98943274e-05 ,2.18710971e-05 ,9.30997631e-06
 ,7.43556341e-06 ,5.60376608e-05 ,1.23196691e-03 ,1.33818940e-02
 ,1.60025000e-01 ,2.26721699e-02 ,7.76596560e-04 ,1.26199945e-04]
 ,[3.42109464e-07 ,1.64253363e-06 ,6.44505446e-06 ,1.60554012e-05
 ,1.70656197e-05 ,1.04493639e-05 ,3.59392793e-06 ,1.38386202e-06
 ,1.06680713e-06 ,8.09129593e-06 ,1.76370103e-04 ,1.91050586e-03
 ,2.26721699e-02 ,1.60025000e-01 ,5.34702220e-03 ,8.82412680e-04]
 ,[1.79267289e-07 ,5.83219780e-07 ,1.09297135e-06 ,1.69329372e-06
 ,9.22409479e-07 ,4.18410356e-07 ,1.23926038e-07 ,4.62476670e-08
 ,3.60909075e-08 ,2.72335799e-07 ,5.97985797e-06 ,6.49571254e-05
 ,7.76596560e-04 ,5.34702220e-03 ,1.60025000e-01 ,2.54867019e-02]
 ,[1.39433490e-07 ,3.58352044e-07 ,4.53545300e-07 ,5.49823838e-07
 ,2.15992686e-07 ,8.29470309e-08 ,2.19426161e-08 ,7.73993834e-09
 ,5.88478995e-09 ,4.46322182e-08 ,9.73782845e-07 ,1.05663376e-05
 ,1.26199945e-04 ,8.82412680e-04 ,2.54867019e-02 ,1.60025000e-01]])

k_sb = np.array([[2.78705243e-74 ,4.66087824e-42 ,1.36682936e-20 ,6.76448472e-10
 ,2.98538547e-07 ,8.33599745e-07 ,3.51482165e-28 ,2.13593989e-60
 ,1.71371970e-66 ,1.26164642e-34 ,1.98582375e-14 ,1.81716391e-08
 ,6.41392850e-09 ,1.60659174e-16 ,5.44654614e-37 ,6.79761558e-69
 ,5.64921193e-54 ,2.64559249e-31 ,2.13941020e-16 ,2.29532799e-09
 ,8.11599983e-08 ,1.92828098e-06 ,5.95936194e-20 ,1.71726812e-42
 ,1.94704878e-52 ,6.55192978e-30 ,2.21916937e-15 ,3.71870831e-09
 ,4.53732960e-08 ,7.90035604e-09 ,4.93223999e-22 ,2.18398251e-44]
 ,[6.85149548e-74 ,1.10922533e-41 ,2.98974419e-20 ,1.19519010e-09
 ,4.38631711e-07 ,2.02058346e-07 ,1.23523825e-28 ,7.94967030e-61
 ,1.14470423e-65 ,9.00829399e-34 ,1.51415667e-13 ,4.32984523e-08
 ,7.34854773e-09 ,4.79464189e-17 ,1.09782200e-37 ,1.23143412e-69
 ,3.49539484e-54 ,1.81876391e-31 ,1.89204803e-16 ,3.31303461e-09
 ,1.55964740e-07 ,1.33318870e-05 ,1.60623631e-19 ,3.77162330e-42
 ,1.25034407e-51 ,3.85237198e-29 ,9.76536384e-15 ,7.53353933e-09
 ,5.79351517e-08 ,1.41840782e-09 ,6.41534243e-23 ,2.92424122e-45]
 ,[1.30872776e-73 ,1.94970921e-41 ,4.23003008e-20 ,9.84859869e-10
 ,2.33434290e-07 ,3.18033651e-08 ,4.47418427e-29 ,3.32154725e-61
 ,2.05099132e-64 ,1.66680301e-32 ,2.39739317e-12 ,5.20824988e-08
 ,3.14600528e-09 ,4.84263181e-18 ,7.74164096e-39 ,7.92688232e-71
 ,9.16787511e-55 ,5.27033269e-32 ,6.96750177e-17 ,1.98793246e-09
 ,1.28037399e-07 ,2.37992586e-04 ,1.00353194e-18 ,1.92711898e-41
 ,1.27081634e-50 ,3.19054026e-28 ,4.26679619e-14 ,7.34312963e-09
 ,2.72250982e-08 ,8.68060835e-11 ,3.73590915e-24 ,1.94254393e-46]
 ,[2.90393430e-73 ,4.01895683e-41 ,7.16064735e-20 ,1.00910213e-09
 ,1.61474981e-07 ,6.06606205e-09 ,1.48046552e-29 ,1.23424159e-61
 ,1.28160428e-63 ,1.10915369e-31 ,1.67539564e-11 ,6.43940643e-08
 ,2.04813184e-09 ,1.29198859e-18 ,1.57704247e-39 ,1.48904687e-71
 ,5.26572673e-55 ,3.16698725e-32 ,4.65126116e-17 ,1.63079986e-09
 ,1.19792613e-07 ,1.35580823e-03 ,2.23041389e-18 ,3.85835303e-41
 ,7.41275253e-50 ,1.65395070e-27 ,1.43981263e-13 ,8.17293978e-09
 ,1.81929186e-08 ,1.49755197e-11 ,5.23165612e-25 ,2.79507444e-47]
 ,[5.60923915e-73 ,6.56481516e-41 ,7.51051130e-20 ,3.72943362e-10
 ,3.00748972e-08 ,4.96019002e-10 ,3.81038740e-30 ,4.12079531e-62
 ,3.01360460e-62 ,2.71903138e-30 ,3.30660140e-10 ,2.83948222e-08
 ,3.90078736e-10 ,9.08785177e-20 ,8.39738993e-41 ,7.31738537e-73
 ,1.12797609e-55 ,6.96965553e-33 ,1.08910513e-17 ,4.30370744e-10
 ,3.41161044e-08 ,1.87706732e-02 ,1.21195110e-17 ,1.96183988e-40
 ,9.23267492e-49 ,1.53532294e-26 ,4.86905102e-13 ,3.26231296e-09
 ,3.43107983e-09 ,6.60186070e-13 ,2.26397663e-26 ,1.38085170e-48]
 ,[1.03702406e-72 ,1.04059872e-40 ,7.90754858e-20 ,1.56960132e-10
 ,7.48153658e-09 ,6.38576513e-11 ,1.11474115e-30 ,1.50521350e-62
 ,2.96249881e-61 ,2.80589622e-29 ,3.03127915e-09 ,1.24410438e-08
 ,1.04829330e-10 ,1.34953442e-20 ,1.03005204e-41 ,8.44393370e-74
 ,4.05580649e-56 ,2.47979243e-33 ,3.78669824e-18 ,1.43687088e-10
 ,1.11249341e-08 ,1.70891826e-02 ,3.15579857e-17 ,5.26554772e-40
 ,6.41293875e-48 ,8.44912413e-26 ,1.13182046e-12 ,1.39744159e-09
 ,8.90913961e-10 ,6.81525853e-14 ,2.17182356e-27 ,1.42073843e-49]
 ,[1.95966625e-72 ,1.61973790e-40 ,7.35878492e-20 ,4.98140901e-11
 ,1.39019910e-09 ,6.72287323e-12 ,2.78353750e-31 ,4.89749413e-63
 ,3.33935400e-60 ,3.33792700e-28 ,3.07149163e-08 ,3.85841912e-09
 ,2.18045366e-11 ,1.71302060e-21 ,1.10039809e-42 ,8.51133447e-75
 ,1.33616619e-56 ,7.85901661e-34 ,1.10500343e-18 ,3.65546597e-11
 ,2.62612620e-09 ,2.36570019e-03 ,7.41742973e-17 ,1.39589455e-39
 ,4.99424225e-47 ,4.97237119e-25 ,2.27957810e-12 ,4.38979634e-10
 ,1.76078195e-10 ,6.02943537e-15 ,1.78947214e-28 ,1.25469144e-50]
 ,[3.89695882e-72 ,2.72331598e-40 ,7.87470041e-20 ,2.12885460e-11
 ,3.89449144e-10 ,1.04303357e-12 ,7.58701508e-32 ,1.64175481e-63
 ,2.03706288e-59 ,2.16806178e-27 ,1.94807564e-07 ,1.55919995e-09
 ,6.81494235e-12 ,3.80922490e-22 ,2.13871674e-43 ,1.57481566e-75
 ,6.73877489e-57 ,3.76393373e-34 ,4.75186977e-19 ,1.32558608e-11
 ,8.72178666e-10 ,3.60542626e-04 ,1.03302022e-16 ,2.30023453e-39
 ,2.75166671e-46 ,2.21961058e-24 ,4.16329306e-12 ,1.82952600e-10
 ,5.23369505e-11 ,9.66577417e-16 ,2.53236373e-29 ,1.81066837e-51]
 ,[8.94531730e-72 ,5.72332057e-40 ,1.27510057e-19 ,1.89021506e-11
 ,2.55630115e-10 ,3.17404540e-13 ,2.53397016e-32 ,5.92836601e-64
 ,4.64464386e-59 ,5.32822294e-27 ,6.43941668e-07 ,1.31014122e-09
 ,4.90160576e-12 ,2.17732959e-22 ,1.10029061e-43 ,7.76777177e-76
 ,7.12127116e-57 ,3.80728490e-34 ,4.38368151e-19 ,1.05539839e-11
 ,6.43495233e-10 ,1.10747312e-04 ,7.34026233e-17 ,1.86737889e-39
 ,9.06239853e-46 ,6.97240929e-24 ,8.77143089e-12 ,1.58470501e-10
 ,3.60993318e-11 ,4.02890975e-16 ,8.46596389e-30 ,5.71460374e-52]
 ,[2.09223725e-71 ,1.59324547e-39 ,5.35765160e-19 ,1.35807356e-10
 ,1.96374418e-09 ,7.72135369e-13 ,2.27157721e-32 ,4.05314213e-64
 ,6.99805514e-60 ,8.59691048e-28 ,2.39575056e-07 ,9.64607040e-09
 ,3.74668210e-11 ,1.64528109e-21 ,8.03360091e-43 ,5.56136936e-75
 ,4.59856897e-56 ,2.53338062e-33 ,3.08144247e-18 ,7.89636887e-11
 ,4.90455427e-09 ,2.42444301e-04 ,1.46481509e-17 ,3.27227620e-40
 ,5.15053426e-46 ,5.82377393e-24 ,2.37203311e-11 ,1.15183354e-09
 ,2.77104835e-10 ,2.47237903e-15 ,3.51291082e-29 ,1.92068600e-51]
 ,[1.27722776e-70 ,1.19968342e-38 ,6.62907467e-18 ,2.98585192e-09
 ,3.95903486e-08 ,1.22515931e-12 ,9.19613459e-33 ,1.17300658e-64
 ,5.24302306e-61 ,7.27370561e-29 ,5.93649294e-08 ,2.13529600e-07
 ,8.03811216e-10 ,3.01800903e-20 ,1.32213273e-41 ,8.70891238e-74
 ,9.48437403e-55 ,5.33704113e-32 ,6.71576160e-17 ,1.74370340e-09
 ,1.06011433e-07 ,1.36760747e-04 ,8.63908028e-19 ,1.74323226e-41
 ,3.80539617e-46 ,7.22124038e-24 ,1.43966334e-10 ,2.54306940e-08
 ,5.80551365e-09 ,2.49087554e-14 ,1.69220513e-28 ,6.59458686e-51]
 ,[4.24145121e-70 ,4.63740027e-38 ,3.79412044e-17 ,3.13825429e-08
 ,4.09141624e-07 ,1.65226280e-12 ,5.08679686e-33 ,5.20895333e-65
 ,6.37171703e-62 ,9.41524088e-30 ,1.26526744e-08 ,2.29002639e-06
 ,8.67055638e-09 ,2.97092100e-19 ,1.22110056e-40 ,7.81910944e-73
 ,9.24787140e-54 ,5.31770065e-31 ,6.97612694e-16 ,1.89047217e-08
 ,1.14651644e-06 ,4.63078444e-05 ,1.03077517e-19 ,1.93648764e-42
 ,2.33440409e-46 ,6.17438159e-24 ,3.86889494e-10 ,2.69900736e-07
 ,6.17503467e-08 ,1.52509070e-13 ,6.21592402e-28 ,1.94190700e-50]
 ,[1.41010634e-69 ,1.76967242e-37 ,2.11293459e-16 ,3.56366079e-07
 ,4.56046676e-06 ,1.81751353e-12 ,2.59431838e-33 ,2.20165624e-65
 ,6.98099736e-63 ,1.08492529e-30 ,2.05923602e-09 ,2.67336879e-05
 ,1.02856644e-07 ,3.18622671e-18 ,1.23175100e-39 ,7.68787199e-72
 ,9.77373763e-53 ,5.73932336e-30 ,7.86755875e-15 ,2.25150767e-07
 ,1.36358409e-05 ,9.20767719e-06 ,1.10751876e-20 ,1.96620627e-43
 ,1.30131387e-46 ,4.57569135e-24 ,8.14171873e-10 ,3.10376194e-06
 ,7.18632371e-07 ,9.23583682e-13 ,2.28127182e-27 ,5.82985093e-50]
 ,[2.88730398e-69 ,3.99373646e-37 ,6.34410480e-16 ,2.20663683e-06
 ,3.13536060e-05 ,2.07430689e-12 ,1.80374885e-33 ,1.35349001e-65
 ,1.11165523e-63 ,1.77007023e-31 ,3.92148233e-10 ,1.71771442e-04
 ,7.25088468e-07 ,2.16744079e-17 ,8.12700594e-39 ,5.00795832e-71
 ,5.72297631e-52 ,3.42623035e-29 ,4.91725691e-14 ,1.53247162e-06
 ,9.60034633e-05 ,2.38673656e-06 ,2.08607100e-21 ,3.55659624e-44
 ,6.35809027e-47 ,2.66629701e-24 ,9.42209921e-10 ,1.95124829e-05
 ,5.05971441e-06 ,4.12478732e-12 ,7.30253926e-27 ,1.64266960e-49]
 ,[1.75565536e-68 ,2.72994699e-36 ,6.18409213e-15 ,6.00210553e-05
 ,6.35272879e-04 ,9.36791983e-13 ,4.50260816e-34 ,2.92348604e-66
 ,6.00900388e-65 ,1.00007221e-32 ,2.77886873e-11 ,4.75525064e-03
 ,2.03809005e-05 ,4.83959060e-16 ,1.66999503e-37 ,9.98937911e-70
 ,1.54291137e-50 ,9.37691599e-28 ,1.39510567e-12 ,4.58479262e-05
 ,2.69279226e-03 ,1.16534044e-07 ,8.32172876e-23 ,1.37779069e-45
 ,3.23210913e-47 ,1.71297954e-24 ,1.48084770e-09 ,5.34331577e-04
 ,1.29183574e-04 ,2.71975254e-11 ,2.72541847e-26 ,5.05772324e-49]
 ,[3.02738896e-68 ,4.94257937e-36 ,1.31448622e-14 ,2.74354162e-04
 ,3.33275652e-03 ,7.91278092e-13 ,3.07227941e-34 ,1.88852135e-66
 ,1.03181102e-65 ,1.73204428e-33 ,5.01484159e-12 ,1.83723335e-02
 ,1.27897606e-04 ,2.92891468e-15 ,9.93343824e-37 ,5.90583449e-69
 ,7.73713462e-50 ,4.75472885e-27 ,7.30407900e-12 ,2.66507457e-04
 ,1.68929362e-02 ,2.62707083e-08 ,1.73321814e-23 ,2.82438299e-46
 ,1.41158549e-47 ,8.07113874e-25 ,9.41955770e-10 ,2.34301441e-03
 ,7.94661339e-04 ,9.38485333e-11 ,7.70202407e-26 ,1.34046936e-48]])

depth = np.array([0. ,0. ,0. ,0. ,0. ,0.
 ,0. ,4.88448419 ,5.32169311 ,9.86217963 ,4.41356831 ,0.
 ,0. ,0. ,0. ,0.  ])


print(k_bb.shape)
print(k_sb.shape)
f_s = np.transpose(k_sb)@ np.linalg.inv(k_bb)@depth
print(f_s)np.transpose(k_sb)@ np.linalg.inv(k_bb)@depth
print(max(f_s))