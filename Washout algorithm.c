//Low-pass acceleration channel
void Tilt()
{
	float  k1, k2, kd, kd1, kd2, c;
		//低通滤波
	  wal = 2.57;
		axd_2 = axd_1;
		axd_1 = axd;
		axd = AX;
		ax_2 = ax_1;
		ax_1 = ax;
		k1 = (8 - 2 * wal*wal*T*T)*ax_1;
		k2 = (4 * zuni_al*wal*T - 4 - wal*wal*T*T) * ax_2;
		kd = (wal*wal*T*T*axd);
		kd1 = (2 * wal*wal*T*T*axd_1);
		kd2 = (wal*wal*T*T*axd_2);
		c = 1 / (4 + 4 * wal*zuni_al*T + wal*wal*T*T);
		ax = c*(k1 + k2 + kd + kd1 + kd2);

		dax_1 = dax;
		wax_1 = wax;
		dax = asin(ax / 9.8);
		wax = (1 / T)*(dax - dax_1);
		if (wax > 0.06283)
		{
			wax = 0.06283;
		}
		else if (wax < -0.06283)
		{
			wax = -0.06283;
		}
		pitch = pitch + ((wax + wax_1) * T) / 2;
		//低通滤波
		wal = 2.7;
		ayd_2 = ayd_1;
		ayd_1 = ayd;
		ayd = AY;
		ay_2 = ay_1;
		ay_1 = ay;
		k1 = (8 - 2 * wal*wal*T*T)*ay_1;
		k2 = (4 * zuni_al*wal*T - 4 - wal*wal*T*T) * ay_2;
		kd = (wal*wal*T*T*ayd);
		kd1 = (2 * wal*wal*T*T*ayd_1);
		kd2 = (wal*wal*T*T*ayd_2);
		c = 1 / (4 + 4 * wal*zuni_al*T + wal*wal*T*T);
		ay = c*(k1 + k2 + kd + kd1 + kd2);

		day_1 = day;
		way_1 = way;
		day = asin(ay / 9.8);
		way = (1 / T)*(day - day_1);
		if (way > 0.05235)
		{
			way = 0.05235;
		}
		else if (way < -0.05235)
		{
			way = -0.05235;
		}
		roll = roll + ((way + way_1) * T) / 2;
}
//High-pass angular velocity channel
//高通角速度通道Wx是控制roll、Wy控制pitch
void Axe()
{
	float  k1, k2, kd, kd1, kd2, c;

		//Filter
    wwh = 1.52;
    zuni_wh = 0.95;
		wxd_2 = wxd_1;
		wxd_1 = wxd;
		wxd = Wx + Wy*sin(Roll)*tan(Pitch) + Wz*cos(Roll)*tan(Pitch);
		wx_2 = wx_1;
		wx_1 = wx;
		k1 = (8 - 2 * wwh*wwh*T*T)*wx_1;
		k2 = (4 * zuni_wh*wwh*T - 4 - wwh*wwh*T*T) * wx_2;
		kd = (4 * wxd);
		kd1 = (-8 * wxd_1);
		kd2 = (4 * wxd_2);
		c = 1 / (4 + 4 * zuni_wh* wwh*T + wwh*wwh*T*T);
		wx = c*(k1 + k2 + kd + kd1 + kd2);
		Roll = Roll + ((wx + wx_1) * T) / 2;

		//Filter
		wwh = 1.3;
		zuni_wh = 0.9;
		wyd_2 = wyd_1;
		wyd_1 = wyd;
		wyd = Wy*cos(Roll) - Wz*sin(Roll);
		wy_2 = wy_1;
		wy_1 = wy;
		k1 = (8 - 2 * wwh*wwh*T*T)*wy_1;
		k2 = (4 * wwh* zuni_wh*T - 4 - wwh*wwh*T*T) * wy_2;
		kd = (4 * wyd);
		kd1 = (-8 * wyd_1);
		kd2 = (4 * wyd_2);
		c = 1 / (4 + 4 * zuni_wh* wwh*T + wwh*wwh*T*T);
		wy = c*(k1 + k2 + kd + kd1 + kd2);
		Pitch = Pitch + ((wy + wy_1) * T) / 2;
		//Filter
		wwh = 1.7;
		zuni_wh = 1;
		wzd_2 = wzd_1;
		wzd_1 = wzd;
		wzd = 0.01*(Wy*sin(Roll)/sin(Pitch) + Wz*cos(Roll)/sin(Pitch));
		wz_2 = wz_1;
		wz_1 = wz;
		k1 = (8 - 2 * wwh*wwh*T*T)*wz_1;
		k2 = (4 * wwh* zuni_wh*T - 4 - wwh*wwh*T*T) * wz_2;
		kd = (4 * wzd);
		kd1 = (-8 * wzd_1);
		kd2 = (4 * wzd_2);
		c = 1 / (4 + 4 * zuni_wh* wwh*T + wwh*wwh*T*T);
		wz = c*(k1 + k2 + kd + kd1 + kd2);
		Yaw = Yaw + ((wz + wz_1) * T) / 2;
		if (Yaw > 0.1)
			Yaw = 0.1;
		if (Yaw < -0.1)
			Yaw = 0.1;
}

//High-pass acceleration channel
void Displacement()
{
	float  k1, k2, kd, kd1, kd2, c;
		vx_1 = vx;
		//Filter1
		wah = 2.83;
		zuni_ah = 0.93;
		axdh_2 = axdh_1;
		axdh_1 = axdh;
		axdh = (AX*cos(Yaw)*cos(Pitch) + AY*(-sin(Yaw)*cos(Roll) + cos(Yaw)*sin(Pitch)*sin(Roll))+AZ*(sin(Yaw)*sin(Roll)+cos(Yaw)*sin(Pitch)*cos(Roll)));
		axh_2 = axh_1;
		axh_1 = axh;
		k1 = (8 - 2 * wah*wah*T*T)*axh_1;
		k2 = (4 * wah*zuni_ah*T - 4 - wah*wah*T*T) * axh_2;
		kd = (4 * axdh);
		kd1 = (-8 * axdh_1);
		kd2 = (4 * axdh_2);
		c = 1 / (4 + 4 * zuni_ah*wah*T + wah*wah*T*T);
		axh = c*(k1 + k2 + kd + kd1 + kd2);

		axdh2_1 = axdh2;
		axdh2 = axh;
		axh2_1 = axh2;
		k1 = (2 - wah0*T)*axh2_1;
		kd = (2 * axdh2);
		kd1 = (-2 * axdh2_1);
		c = 1/(2 + wah0*T);
		axh2 = c*(k1 + kd + kd1);
		vx = vx + (T / 2)*(axh2 + axh2_1);
		X = X + (T / 2)*(vx_1 + vx);
		wah = 2.92+3;
		zuni_ah = 0.9;
		vy_1 = vy;
		//Filter2
		aydh_2 = aydh_1;
		aydh_1 = aydh;
		aydh = 0.1*(AX*sin(Yaw)*cos(Pitch) +AY*(cos(Yaw)*cos(Roll) + sin(Yaw)*sin(Pitch)*sin(Roll)) +AZ*(-cos(Yaw)*sin(Roll) + sin(Yaw)*sin(Pitch)*cos(Roll)));
		ayh_2 = ayh_1;
		ayh_1 = ayh;
		k1 = (8 - 2 * wah*wah*T*T)*ayh_1;
		k2 = (4 * wah*zuni_ah*T - 4 - wah*wah*T*T) * ayh_2;
		kd = (4 * aydh);
		kd1 = (-8 * aydh_1);
		kd2 = (4 * aydh_2);
		c = 1 / (4 + 4 * zuni_ah*wah*T + wah*wah*T*T);
		ayh = c*(k1 + k2 + kd + kd1 + kd2);

		aydh2_1 = aydh2;
		aydh2 = ayh;
		ayh2_1 = ayh2;
		k1 = (2 - wah0*T)*ayh2_1;
		kd = (2 * aydh2);
		kd1 = (-2 * aydh2_1);
		c =1/( 2 + wah0*T);
		ayh2 = c*(k1 + kd + kd1);
		vy = vy + (T / 2)*(ayh2 + ayh2_1);
		Y = Y + (T / 2)*(vy_1 + vy);

		wah = 3.2+3;
		zuni_ah = 0.95;
		vz_1 = vz;
		//Filter3
		azdh_2 = azdh_1;
		azdh_1 = azdh;
		azdh = 0.1*(-sin(Pitch)*AX +AY*cos(Pitch)*sin(Roll) + AZ*cos(Pitch)*cos(Roll));
		azh_2 = azh_1;
		azh_1 = azh;
		k1 = (8 - 2 * wah*wah*T*T)*azh_1;
		k2 = (4 * wah*zuni_ah*T - 4 - wah*wah*T*T) * azh_2;
		kd = (4 * azdh);
		kd1 = (-8 * azdh_1);
		kd2 = (4 * azdh_2);
		c = 1 / (4 + 4 * zuni_ah*wah*T + wah*wah*T*T);
		azh = c*(k1 + k2 + kd + kd1 + kd2);
		azdh2_1 = azdh2;
		azdh2 = azh;
		azh2_1 = azh2;
		k1 = (2 - wah0*T)*azh2_1;
		kd = (2 * azdh2);
		kd1 = (-2 * azdh2_1);
		c = 1/(2 + wah0*T);
		azh2 = c*(k1 + kd + kd1);
		vz = vz + (T / 2)*(azh2 + azh2_1);
		Z = Z + (T / 2)*(vz_1 + vz);
}
