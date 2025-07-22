import pandas as pd
import numpy as np

# Load CSV with header (first row is header by default)
df = pd.read_csv("Data2.csv")

# Extract each column as a separate array
Alpha_L1 = np.deg2rad(df.iloc[:, 1].to_numpy())
Alpha_L2 = np.deg2rad(df.iloc[:, 2].to_numpy())
Alpha_R1 = np.deg2rad(df.iloc[:, 3].to_numpy())
Alpha_R2 = np.deg2rad(df.iloc[:, 4].to_numpy())
Ay      = df.iloc[:, 5].to_numpy()  # lateral acceleration
Camber_L1 = np.deg2rad(df.iloc[:, 6].to_numpy())
Camber_L2 = np.deg2rad(df.iloc[:, 7].to_numpy())
Camber_R1 = np.deg2rad(df.iloc[:, 8].to_numpy())
Camber_R2 = np.deg2rad(df.iloc[:, 9].to_numpy())
Fy_L1 = df.iloc[:, 10].to_numpy()
Fy_L2 = df.iloc[:, 11].to_numpy()
Fy_R1 = df.iloc[:, 12].to_numpy()
Fy_R2 = df.iloc[:, 13].to_numpy()
Roll = df.iloc[:, 14].to_numpy()

Camber_Front = (Camber_L1 + Camber_R1)/2
Camber_Rear  = (Camber_L2 + Camber_R2)/2
Alpha_Front  = (Alpha_L1 + Alpha_R1)/2
Alpha_Rear   = (Alpha_L2 + Alpha_R2)/2

Fy_f         = Fy_L1 + Fy_R1
Fy_r         = Fy_L2 + Fy_R2

R            =  24 #m

#==============================================================================

#Use linear regression to find the Cornering Stiffness
coeffs_alphaF = np.polyfit(Alpha_Front, Fy_f, 1)  # 1st degree poly (linear fit)
coeffs_alphaR = np.polyfit(Alpha_Rear, Fy_r, 1)  # 1st degree poly (linear fit)

#Extracting the the gradient
slope_alphaF = coeffs_alphaF[0]
slope_alphaR = coeffs_alphaR[0]

# since Fy = -C_alpha * alpha
C_alphaF = -slope_alphaF
C_alphaR = -slope_alphaR

#Use linear regression to find the Camber Stiffness
coeffs_camberF = np.polyfit(Camber_Front, Fy_f, 1)  # 1st degree poly (linear fit)
#coeffs_camberR = np.polyfit(Camber_Rear, Fy_r, 1)  # 1st degree poly (linear fit)

#Extracting the the gradient
C_camberF = coeffs_camberF[0]
#C_camberR = coeffs_camberR[0]

#================================================================================

#Camber Gradient calculation
coeffs_cambergradF = np.polyfit(Roll, Camber_Front, 1)  # 1st degree poly (linear fit)
#coeffs_cambergradR = np.polyfit(Roll, Camber_Rear, 1)  # 1st degree poly (linear fit)

#Extracting the the gradient
cambergradF = coeffs_cambergradF[0]
#cambergradR = coeffs_cambergradR[0]

#Rate of change of roll angle w.r.t to acceleration in lateral direction
coeffs_rollgrad = np.polyfit(Ay, Roll, 1)  # 1st degree poly (linear fit)
rollgrad = coeffs_rollgrad[0]

#================================================================================

#Calculating understeer coefficient
K_camber  = (C_camberF/C_alphaF)*cambergradF*rollgrad

print("Understeer coefficient:", K_camber)