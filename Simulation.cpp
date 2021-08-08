#include "Simulation.h"
#include <cstdio>
#include <algorithm>
#include "mpi.h"
#include "omp.h"

CSimulation::CSimulation(int simx, int simy, int simz, double f, double thr, DistanceType distanceType, SearchArea searchArea, SimulatingPathType simulPathType, bool isHaveSamples)
{
	m_SimX = simx;
	m_SimY = simy;
	m_SimZ = simz;
	m_F = f;
	m_T = thr;
	m_SearchArea = searchArea;
	m_DistanceType = distanceType;
	m_Simul_Path_Type = simulPathType;
	IsHaveSamples = isHaveSamples;
	IsSimulation = false;
	m_TiX = m_TiY = m_TiZ = 0;
	IsHaveTi = false;

	m_Sim.clear();
	m_SimMark.clear();
	p_PathSim.clear();
	sim_path.clear();
	for (int i = 0; i < m_SimZ; i++)
	{
		vector<vector<double>> temp;
		vector<vector<bool>> tempMark;
		for (int j = 0; j < m_SimX; j++)
		{
			vector<double> temp0;
			vector<bool> tempMark0;
			for (int k = 0; k < m_SimY; k++)
			{
				temp0.push_back(0);
				tempMark0.push_back(false);
			}
			temp.push_back(temp0);
			tempMark.push_back(tempMark0);
		}
		m_Sim.push_back(temp);
		m_SimMark.push_back(tempMark);
	}
	p_ScanCouts = new int[m_SimX*m_SimY*m_SimZ];
	p_Bestmin = new int[m_SimX*m_SimY*m_SimZ];
}

void CSimulation::SetParameter(int simx, int simy, int simz, double f, double thr, DistanceType distanceType, SearchArea searchArea, SimulatingPathType simulPathType, bool isHaveSamples)
{
	m_SimX = simx;
	m_SimY = simy;
	m_SimZ = simz;
	m_F = f;
	m_T = thr;
	m_SearchArea = searchArea;
	m_DistanceType = distanceType;
	m_Simul_Path_Type = simulPathType;
	IsHaveSamples = isHaveSamples;

	m_Sim.clear();
	m_SimMark.clear();
	p_PathSim.clear();
	for (int i = 0; i < m_SimZ; i++)
	{
		vector<vector<double>> temp;
		vector<vector<bool>> tempMark;
		for (int j = 0; j < m_SimX; j++)
		{
			vector<double> temp0;
			vector<bool> tempMark0;
			for (int k = 0; k < m_SimY; k++)
			{
				temp0.push_back(0);
				tempMark0.push_back(false);
			}
			temp.push_back(temp0);
			tempMark.push_back(tempMark0);
		}
		m_Sim.push_back(temp);
		m_SimMark.push_back(tempMark);
	}
	p_ScanCouts = new int[m_SimX*m_SimY*m_SimZ];
	p_Bestmin = new int[m_SimX*m_SimY*m_SimZ];
}

bool CSimulation::LoadTi(string FilePath)
{
	m_Property.MinValue = TMP_MAX;
	m_Property.MaxValue = INT8_MIN;

	m_Ti.clear();
	m_TiX = m_TiY = m_TiZ = 0;
	IsHaveTi = false;

	ifstream ifile;
	ifile.open(FilePath);
	if (!ifile)
	{
		std::cout << "ti not exist" << std::endl;
		return false;
	}

	string temp;
	ifile >> m_TiX >> m_TiY >> m_TiZ;
	ifile >> temp;
	if (temp != "1")
	{
		return false;
	}
	ifile >> m_Property.PropertyName;

	for (int k = 0; k < m_TiZ; k++)
	{
		vector<vector<double>> xyTi;
		for (int i = 0; i < m_TiX; i++)
		{
			vector<double> yTi;
			for (int j = 0; j < m_TiY; j++)
			{
				double values;
				ifile >> values;
				m_Property.MinValue = m_Property.MinValue < values ? m_Property.MinValue : values;
				m_Property.MaxValue = m_Property.MaxValue > values ? m_Property.MaxValue : values;
				yTi.push_back(values);
			}
			xyTi.push_back(yTi);
		}
		m_Ti.push_back(xyTi);
	}
	ifile.close();
	IsHaveTi = true;
	return true;
}

bool CSimulation::LoadSamples(string FilePath)
{
	m_Samples.clear();
	m_SamplesMinX = m_SamplesMinY = m_SamplesMinZ = INT8_MAX;
	m_SamplesMaxX = m_SamplesMaxY = m_SamplesMaxZ = -1;

	ifstream ifile;
	ifile.open(FilePath);
	if (!ifile)
	{
		return false;
	}

	string temp;
	int Couts;
	ifile >> Couts;
	ifile >> temp;
	if (temp != "4")
	{
		return false;
	}
	ifile >> temp;
	if (temp != "x")
	{
		return false;
	}
	ifile >> temp;
	if (temp != "y")
	{
		return false;
	}
	ifile >> temp;
	if (temp != "z")
	{
		return false;
	}
	ifile >> temp;
	if (temp == "")
	{
		return false;
	}

	for (int i = 0; i < Couts; i++)
	{
		C3DPoint tempPoint;
		ifile >> tempPoint.x >> tempPoint.y >> tempPoint.z >> tempPoint.values;
		tempPoint.x--;
		tempPoint.y--;
		tempPoint.z--;
		m_Samples.push_back(tempPoint);

		m_SamplesMinX = m_SamplesMinX < tempPoint.x ? m_SamplesMinX : tempPoint.x;
		m_SamplesMaxX = m_SamplesMaxX > tempPoint.x ? m_SamplesMaxX : tempPoint.x;
		m_SamplesMinY = m_SamplesMinY < tempPoint.y ? m_SamplesMinY : tempPoint.y;
		m_SamplesMaxY = m_SamplesMaxY > tempPoint.y ? m_SamplesMaxY : tempPoint.y;
		m_SamplesMinZ = m_SamplesMinZ < tempPoint.z ? m_SamplesMinZ : tempPoint.z;
		m_SamplesMaxZ = m_SamplesMaxZ > tempPoint.z ? m_SamplesMaxZ : tempPoint.z;
	}
	ifile.close();
	IsHaveSamples = true;
	return true;
}

bool CSimulation::SaveSimulation(string FilePath, PropertyFileType ext)
{
	
	ofstream ofile;
	ofile.open(FilePath);
	if (!ofile)
	{
		return false;
	}
	switch (ext)
	{
	case SGEMS:
		{
			ofile << m_SimX << " " << m_SimY << " " << m_SimZ << endl;
			ofile << "1" << endl;
			ofile << m_Property.PropertyName << endl;
			for (int k = 0; k < m_SimZ; k++)
			{
				for (int i = 0; i < m_SimX; i++)
				{
					for (int j = 0; j < m_SimY; j++)
					{
						ofile << m_Sim[k][i][j] << endl;
					}
				}
			}
			break;
		}
	case ATT:
		{
			int ni, nj;
			ni = nj = 0;
			int temp = 0;
			ofile << "-- Proper name : ";
			ofile << m_Property.PropertyName << endl;
			ofile << m_Property.PropertyName << endl;
			for (int k = m_SimZ-1; k >= 0; k--)
			{
				for (int i = 0; i < m_SimX; i++)
				{
					for (int j = 0; j < m_SimY; j++)
					{
						ofile << m_Sim[k][i][j] << " ";
						ni++;
						if (ni == 4)
						{
							ofile << "\n";//¿ØÖÆÃ¿ÐÐÊä³ö4žö
							ni = 0;
							nj++;
						}
						temp = m_SimY - nj * 4;
						if (temp < 4 && temp>0)
						{
							j++;
							for (; j < m_SimY; j++)
							{
								ofile << m_Sim[k][i][j] << " ";//¿ØÖÆÃ¿²ã×îºóÊä³öµÄÖµ
							}
							ofile << "\n";
							nj = 0;
						}
					}
				}
			}
			ofile << "/" << endl;
			return true;
			break;
		}
	default:
		break;
	}
	ofile.close();
	return true;
	
}
//TODO this function should be changed to MPI function
std::string CSimulation::StartSimulation()
{
	//TODO if effectivePoint.size != 0, there may exist conflits
	//compare with Usim(simulating node in slave processes)
    int current_pid, comm_size, nameLen;
    char processorName[MPI_MAX_PROCESSOR_NAME];
	MPI_Comm_rank(MPI_COMM_WORLD, &current_pid);
	MPI_Comm_size(MPI_COMM_WORLD, &comm_size);
	MPI_Get_processor_name(processorName, &nameLen);
	MPI_Status status;
	MPI_Request request; // non-blocking send & recv
	
	srand((unsigned)time(NULL) + current_pid*comm_size + nameLen*1000);

	//init Usim (record the sim_index of current simulation processes)
	std::vector<int> Usim;
	
	//commit C3DPoint Type
	C3DPoint point;
	int blocklens_array[5];
    MPI_Datatype old_type[5];
    MPI_Aint dlp_array[5];
    blocklens_array[0] = 1;
    blocklens_array[1] = 1;
    blocklens_array[2] = 1;
    blocklens_array[3] = 1;
    blocklens_array[4] = 1;
    old_type[0] = MPI_INT;
    old_type[1] = MPI_INT;
    old_type[2] = MPI_INT;
    old_type[3] = MPI_DOUBLE;
    old_type[4] = MPI_DOUBLE;
    MPI_Address(&point.x,&dlp_array[0]);
    MPI_Address(&point.y,&dlp_array[1]);
    MPI_Address(&point.z,&dlp_array[2]);
    MPI_Address(&point.values, &dlp_array[3]);
    MPI_Address(&point.distances, &dlp_array[4]);
    dlp_array[4]=dlp_array[4]-dlp_array[0];
    dlp_array[3]=dlp_array[3]-dlp_array[0];
    dlp_array[2]=dlp_array[2]-dlp_array[0];
    dlp_array[1]=dlp_array[1]-dlp_array[0];
    dlp_array[0]=0;
    MPI_Datatype C3DPointType;
    MPI_Type_struct(5, blocklens_array, dlp_array, old_type, &C3DPointType);                                                 
    MPI_Type_commit(&C3DPointType);
	//declaration of effective point
	vector<C3DPoint> EffectivePoint;
	
	int cntRandomDebug = 0;
	int term_signal;
	
	//pack ti
	int buffer_size;
	double *buffer_for_packed = NULL;

	vector<int> EffectivePointParams;
	
	if(current_pid == 0){
		//std::cout << _x0 << " " << _x1 << " " << _y0 << " " << _y1 << " " << _z0 << " " << _z1 << std::endl;
		_x0 = 0;
		_x1 = 0;
 		_y0 = 0;
    _y1 = 0;
 		_z0 = 0;
    _z1 = 0;
		int xRadius = 0;
		int yRadius = 0;
		int zRadius = 0;

		switch (m_SearchArea.MySearchAreaType)
		{
			case SearchArea::SearchAreaType::Cuboid:
			{
				int xtemp = m_SearchArea.MySearchAreaData.Cuboid.HalfX;
				int ytemp = m_SearchArea.MySearchAreaData.Cuboid.HalfY;
				int ztemp = m_SearchArea.MySearchAreaData.Cuboid.HalfZ;
				xRadius = xtemp < (m_TiX - 1) / 2 ? xtemp : (m_TiX - 1) / 2;
				yRadius = ytemp < (m_TiY - 1) / 2 ? ytemp : (m_TiY - 1) / 2;
				zRadius = ztemp < (m_TiZ - 1) / 2 ? ztemp : (m_TiZ - 1) / 2;
				break;
			}
			case SearchArea::SearchAreaType::Sphere:
			{
				int temp = m_SearchArea.MySearchAreaData.Sphere.SearchRadius;
				xRadius = temp < (m_TiX - 1) / 2 ? temp : (m_TiX - 1) / 2;
				yRadius = temp < (m_TiY - 1) / 2 ? temp : (m_TiY - 1) / 2;
				zRadius = temp < (m_TiZ - 1) / 2 ? temp : (m_TiZ - 1) / 2;
				break;
			}
			default:
				break;
		}
		SetPath_Simul(xRadius,yRadius,zRadius);
		std::cout << "sim path create finish" << std::endl;
		int all_sim_node_num = sim_path.size();
		
		//send Ti
		/*buffer_size = m_TiZ * m_TiX * m_TiY * sizeof(double);
		buffer_for_packed = (double *)malloc(buffer_size);
		int position = 0;
		for(int ii = 0; ii < m_TiZ; ii++) {
			for(int jj = 0; jj < m_TiX; jj++) {
				MPI_Pack(&m_Ti[ii][jj][0], m_TiY, MPI_DOUBLE, buffer_for_packed, buffer_size, &position, MPI_COMM_WORLD);
			}
		}
		std::cout << "TI packed" << std::endl;
		//for (int z = 0; z< m_TiZ; i++)
		vector<int> TiParas;
		TiParas.push_back(m_TiX);
		TiParas.push_back(m_TiY);
		TiParas.push_back(m_TiZ);
		std::cout << TiParas[0] << TiParas[1] << TiParas[2]<<std::endl;
		for(int i = 1; i < comm_size; i++){	
			MPI_Send(&TiParas[0], 3, MPI_INT, i, 55, MPI_COMM_WORLD);
			//MPI_Send(&m_Ti[0][0][0], 1, mpi_cube_type, i, 55, MPI_COMM_WORLD);
			//std::cout << "send Ti to " << i << std::endl;
			MPI_Send(buffer_for_packed, buffer_size, MPI_PACKED, i, 56, MPI_COMM_WORLD);
		}
		std::cout << "send TI finish" << std::endl;*/	
		//init slave data
		for (int i = 1; i < comm_size; i++) {
			int x = -1;
			int currentX = -1, currentY = -1, currentZ = -1;
			//vector<C3DPoint> EffectivePoint;
			while(sim_path.size() != 0) {
				x = sim_path.front();//get a node from sim_path
				vector<int>::iterator k = sim_path.begin();
                		sim_path.erase(k); //remove this node from sim_path
				
				PathToXYZ(x, currentX, currentY, currentZ, m_SimX, m_SimY);
				EffectivePoint.clear();
				GetEffectivePoint(EffectivePoint, currentX, currentY, currentZ);//Get Effective Points
				EffectivePointParams.clear();
                        	EffectivePointParams.push_back(_x0);
                        	EffectivePointParams.push_back(_x1);
                        	EffectivePointParams.push_back(_y0);
                        	EffectivePointParams.push_back(_y1);
                        	EffectivePointParams.push_back(_z0);
                        	EffectivePointParams.push_back(_z1);
				if(EffectivePoint.size() != 0) {
					break; //the N(x) is effective, then break and send data to slave
				}
				else{
					m_Sim[currentZ][currentX][currentY] = m_Ti[rand() % m_TiZ][rand() % m_TiX][rand() % m_TiY];
				}
			}
			//update Usim (use to judge if there is conflict)
			Usim.push_back(x);
			term_signal = 0;
			MPI_Send(&term_signal, 1, MPI_INT, i, 66, MPI_COMM_WORLD);
			int thr_index = all_sim_node_num - sim_path.size();
			EffectivePointParams.push_back(thr_index);
			EffectivePointParams.push_back(x);
			int e_size = EffectivePoint.size();
			EffectivePointParams.push_back(e_size);
      MPI_Send(&EffectivePointParams[0], 9, MPI_INT, i, 1001, MPI_COMM_WORLD);
			MPI_Send(&EffectivePoint[0], e_size, C3DPointType, i, 999, MPI_COMM_WORLD);
		}
		std::cout << "init slaves value finished " << std::endl;
		clock_t staTime, endTime;
		int current_node_index; //index of current node to be simulated
		int p_iter = 1; // iter of processes in current comm 
		int simulated_node_index; //index of simulated node got from slaves
		vector<double> recv_result;
		recv_result.resize(2);
		int lastProgress = 0;
		staTime = clock();
		while (!sim_path.empty()){
			double value = -1;
			MPI_Request request;
			MPI_Irecv(&recv_result[0], 2, MPI_DOUBLE, MPI_ANY_SOURCE, 888, MPI_COMM_WORLD, &request);
			MPI_Wait(&request, &status);
			value = recv_result[0];
			simulated_node_index = (int) recv_result[1];
			p_iter = status.MPI_SOURCE;
			//std::cout << "main process recv slave " << p_iter << " value " << value << " of NO." << simulated_node_index << std::endl;
			//recv VALUE and 3D-INDEX and set them into SG
			int simulated_x, simulated_y, simulated_z;
			PathToXYZ(simulated_node_index, simulated_x, simulated_y, simulated_z, m_SimX, m_SimY);
			m_Sim[simulated_z][simulated_x][simulated_y] = value;
			//update Usim (remove recv node value)
			std::vector<int>::iterator pos;
			pos = find(Usim.begin(), Usim.end(), simulated_node_index);
			if (pos != Usim.end()) {
				Usim.erase(pos);
			}
			//get a new node need to be simulated
			current_node_index = -1;
			//vector<C3DPoint> EffectivePoint;
	        	while(sim_path.size() != 0) {
				int currentX = -1, currentY = -1, currentZ = -1;
				current_node_index = sim_path.front();//get a node from sim_path
				vector<int>::iterator k = sim_path.begin();
				sim_path.erase(k); //remove this node from sim_path
				EffectivePoint.clear();
				PathToXYZ(current_node_index, currentX, currentY, currentZ, m_SimX, m_SimY);
				GetEffectivePoint(EffectivePoint, currentX, currentY, currentZ);//Get Effective Points
				EffectivePointParams.clear();
				EffectivePointParams.push_back(_x0);
                        	EffectivePointParams.push_back(_x1);
                        	EffectivePointParams.push_back(_y0);
                        	EffectivePointParams.push_back(_y1);
                        	EffectivePointParams.push_back(_z0);
                        	EffectivePointParams.push_back(_z1);
				//conflict judgement
				int cntForConflict = 0;
				bool conflict = isConflictExists(EffectivePoint, Usim, currentX, currentY, currentZ);
				/*while(conflict && cntForConflict < 5 && sim_path.size()!=0) {
					sim_path.push_back(current_node_index);
					EffectivePoint.clear();
					current_node_index = sim_path.front();
					vector<int>::iterator temp_iter = sim_path.begin();
					sim_path.erase(temp_iter);
					PathToXYZ(current_node_index, currentX, currentY, currentZ, m_SimX, m_SimY);
                    			GetEffectivePoint(EffectivePoint, currentX, currentY, currentZ);
					EffectivePointParams.clear();
					EffectivePointParams.push_back(_x0);
                        		EffectivePointParams.push_back(_x1);
                       			EffectivePointParams.push_back(_y0);
                       			EffectivePointParams.push_back(_y1);
                        		EffectivePointParams.push_back(_z0);
                        		EffectivePointParams.push_back(_z1);
					conflict = isConflictExists(EffectivePoint, Usim, currentX, currentY, currentZ);
					cntForConflict++;
				}
				if(conflict) {
					//std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
					//sim_path.push_back(current_node_index);
				}*/
				if(EffectivePoint.size() != 0) {
					break; //the N(x) is effective, then break and send data to slave
        }
				else{
					m_Sim[currentZ][currentX][currentY] = m_Ti[rand() % m_TiZ][rand() % m_TiX][rand() % m_TiY];
				}
			}
			//update Usim
			Usim.push_back(current_node_index);
			term_signal = 0;
			MPI_Send(&term_signal, 1, MPI_INT, p_iter, 66, MPI_COMM_WORLD); 
			int thr_index = all_sim_node_num - sim_path.size();
			EffectivePointParams.push_back(thr_index);
			EffectivePointParams.push_back(current_node_index);
			int e_size = EffectivePoint.size();
			EffectivePointParams.push_back(e_size);
			MPI_Send(&EffectivePointParams[0], 9, MPI_INT, p_iter, 1001, MPI_COMM_WORLD);
			//std::cout << "main process send NO." << current_node_index << " to slave " << p_iter << std::endl;
			MPI_Send(&EffectivePoint[0], e_size, C3DPointType, p_iter, 999, MPI_COMM_WORLD);
			int allNum = m_SimX * m_SimY * m_SimZ;
			int nodeSim = allNum - sim_path.size();
			int progress = (int)((nodeSim / (float)allNum) * 100); 
			if (progress % 10 == 0 && progress != lastProgress){
				lastProgress = progress;
				std::cout<< "simulation progress: " << progress << "%" << std::endl;
			}
				//std::cout<< "sim path size: " << sim_path.size() << std::endl;
		}
		//recv the final batch of simulated nodes from slaves
		for(int i = 1; i < comm_size; i++){
			double r_value;
			int r_index;
			//recv sim value
			MPI_Request request;
			vector<double> result;
			result.resize(2); 
			MPI_Irecv(&result[0], 2, MPI_DOUBLE, i, 888, MPI_COMM_WORLD, &request);
			MPI_Wait(&request, &status);
			r_value = result[0];
			r_index = (int) result[1];
			int xx, yy, zz;
			PathToXYZ(r_index, xx, yy, zz, m_SimX, m_SimY);
			m_Sim[zz][xx][yy] = r_value;
			term_signal = 1;
			MPI_Send(&term_signal, 1, MPI_INT, i, 66, MPI_COMM_WORLD);
			result.clear();
		}
		//endTime = clock();
		//std::cout << "The runtime: " << (double) (endTime - staTime) / CLOCKS_PER_SEC << "s" << std::endl;

	}
	//slaves: waiting for the N(x), x, f(x). EffectivePoint, current node, conflict flag.
	else { // slaves
		//recv packed Ti
		/*vector<int> tiParas_slave;
		tiParas_slave.resize(3);
		MPI_Recv(&tiParas_slave[0], 3, MPI_INT, 0, 55, MPI_COMM_WORLD, &status);
		m_TiZ = tiParas_slave[2];
		m_TiY = tiParas_slave[1];
		m_TiX = tiParas_slave[0];
		//std::cout<< m_TiZ <<" "<< m_TiX << " " << m_TiY << std::endl;
		buffer_size = m_TiZ * m_TiX * m_TiY * sizeof(double);
		buffer_for_packed = (double *)malloc(buffer_size);
		int position = 0;
		MPI_Recv(buffer_for_packed, buffer_size, MPI_PACKED, 0, 56, MPI_COMM_WORLD, &status);
		//std::cout << "pack recv" << std::endl;
		//init TI
		m_Ti.resize(m_TiZ);
		for(int k = 0; k < m_TiZ; k++){
			m_Ti[k].resize(m_TiX);
			for(int j = 0; j < m_TiX; j++) {
				m_Ti[k][j].resize(m_TiY);
			}
		}
		//unpack ti
		for (int ii = 0; ii < m_TiZ; ii++){
			for(int jj = 0; jj < m_TiX; jj++) {
				MPI_Unpack(buffer_for_packed, buffer_size, &position, &m_Ti[ii][jj][0], m_TiY, MPI_DOUBLE, MPI_COMM_WORLD);
			}
		}
		//std::cout << "ti recv" << std::endl;*/
		//random variable value
		srand((unsigned)time(NULL) + current_pid*comm_size + nameLen);
		int *pathTi = NULL;

		int xRadius = 0;                                                
		int yRadius = 0;
		int zRadius = 0;

		switch (m_SearchArea.MySearchAreaType)
		{
			case SearchArea::SearchAreaType::Cuboid:
				{
					int xtemp = m_SearchArea.MySearchAreaData.Cuboid.HalfX;
					int ytemp = m_SearchArea.MySearchAreaData.Cuboid.HalfY;
					int ztemp = m_SearchArea.MySearchAreaData.Cuboid.HalfZ;
					xRadius = xtemp < (m_TiX - 1) / 2 ? xtemp : (m_TiX - 1) / 2;
					yRadius = ytemp < (m_TiY - 1) / 2 ? ytemp : (m_TiY - 1) / 2;
					zRadius = ztemp < (m_TiZ - 1) / 2 ? ztemp : (m_TiZ - 1) / 2;
					break;
				}
			case SearchArea::SearchAreaType::Sphere:
				{
					int temp = m_SearchArea.MySearchAreaData.Sphere.SearchRadius;
					xRadius = temp < (m_TiX - 1) / 2 ? temp : (m_TiX - 1) / 2;
					yRadius = temp < (m_TiY - 1) / 2 ? temp : (m_TiY - 1) / 2;
					zRadius = temp < (m_TiZ - 1) / 2 ? temp : (m_TiZ - 1) / 2;
					break;
				}
			default:
				break;
		}
		int scanX = m_TiX - xRadius * 2;
		int scanY = m_TiY - yRadius * 2;
		int scanZ = m_TiZ - zRadius * 2;
		if (m_SearchArea.MySearchAreaType == SearchArea::SearchAreaType::Sphere)
		{
			if (m_SearchArea.MySearchAreaData.Sphere.IsUseSamePathSize == false)
			{
				scanX = m_TiX;
				scanY = m_TiY;
				scanZ = m_TiZ;
			}
		}
		/*int scanX = m_TiX;
		int scanY = m_TiY;
		int scanZ = m_TiZ;*/
		
		//get path of scanning TI
		pathTi = SetPath_TI(scanX, scanY, scanZ); 
		//std::cout << "DEBUG: path ti completed" << std::endl; 
		int j = 0;
		double Thr = 0;
		int cnt_for_debug = 0;
		int thr_index;
		while(1){
			double sim_value = -1;

			int xTemp, yTemp, zTemp;
			int nodeX, nodeY, nodeZ;		
			MPI_Recv(&term_signal, 1, MPI_INT, 0, 66, MPI_COMM_WORLD, &status);
			if(term_signal == 1){
				break;
			}
			//recv node information
			int sim_node_index;
			int recv_length = 0;
			EffectivePointParams.resize(9);
			MPI_Recv(&EffectivePointParams[0], 9, MPI_INT, 0, 1001, MPI_COMM_WORLD, &status);
			_x0 = EffectivePointParams[0];
			_x1 = EffectivePointParams[1];
			_y0 = EffectivePointParams[2];
			_y1 = EffectivePointParams[3];
			_z0 = EffectivePointParams[4];
			_z1 = EffectivePointParams[5];
			thr_index = EffectivePointParams[6];
			sim_node_index = EffectivePointParams[7];
			recv_length = EffectivePointParams[8];
			EffectivePoint.resize(recv_length);
			PathToXYZ(sim_node_index, nodeX, nodeY, nodeZ, m_SimX, m_SimY);
			MPI_Recv(&EffectivePoint[0], recv_length, C3DPointType, 0, 999, MPI_COMM_WORLD, &status);

			if (m_SearchArea.MySearchAreaType == SearchArea::SearchAreaType::Sphere)
			{
				if (m_SearchArea.MySearchAreaData.Sphere.IsUseSamePathSize == false)
				{
					xTemp = m_TiX - _x0 - _x1;
					yTemp = m_TiY - _y0 - _y1;
					zTemp = m_TiZ - _z0 - _z1;
				}
			}
			volatile bool finish_flag = false, assign_flag = false;
      vector<int> bestX, bestY, bestZ;
      vector<double> mindist;
      int numThreads = 3;
      bestX.resize(numThreads);
      bestY.resize(numThreads);
      bestZ.resize(numThreads);
			mindist.resize(numThreads);
      for(int i = 0; i < numThreads; i++ ) {
        bestX[i] = -1;
        bestY[i] = -1;
        bestZ[i] = -1;
        mindist[i] = TMP_MAX;
      }
			//std::cout << "init multiple thread" << std::endl;
			#pragma omp parallel num_threads(3) default(none) shared(mindist, finish_flag, assign_flag, bestX, bestY, bestZ, scanX, scanY, scanZ, std::cout) firstprivate(EffectivePoint, pathTi, j, _x0, _y0, _z0, _x1, _y1, _z1, m_TiX, m_TiY, m_TiZ, xRadius, yRadius, zRadius, nodeX, nodeY, nodeZ)
			{
				int privateBestX = -1, privateBestY = -1, privateBestZ = -1;
				double privateMinDist = TMP_MAX;
				double distances = 0;
				int xxx = scanX*scanY*scanZ*m_F;
				int xTi = -1, yTi = -1, zTi = -1, tempTi = -1;
				int athnum = 3;
				int cthid = omp_get_thread_num();
				int len = xxx / athnum;
				int start = cthid * len;
				int end = cthid * len + len;
				if (cthid == athnum - 1)
				{
					end = xxx;
				}
				//std::cout<<"before for"<<std::endl;
				#pragma omp for
				for (int m = start; m < end; m++)
				{
					j++;
					if(finish_flag == true){
						continue;
					}else{
						if(j >= scanX*scanY*scanZ)
						{
							j = 0;
						}
						if (m_SearchArea.MySearchAreaType == SearchArea::SearchAreaType::Sphere && m_SearchArea.MySearchAreaData.Sphere.IsUseSamePathSize == false){
              xTi = pathTi[j] / (scanY * scanZ);
              tempTi = pathTi[j] % (scanY * scanZ);
              yTi = tempTi / scanZ;
              zTi = tempTi % scanZ;
              if (xTi < _x0 || xTi >= m_TiX - _x1||yTi < _y0 || yTi >= m_TiY - _y1||zTi < _z0 || zTi >= m_TiZ - _z1)
              {
                continue;
              }
						}
						else
            {	
              xTi = pathTi[j] / (scanY * scanZ) + xRadius;
              tempTi = pathTi[j] % (scanY * scanZ);
              yTi = tempTi / scanZ + yRadius;
              zTi = tempTi % scanZ + zRadius;
            }
						distances = GetDistances(EffectivePoint, xTi, yTi, zTi, nodeX, nodeY, nodeZ);
						if (distances == -1) 
						{
							privateMinDist = 99;
							privateBestX = rand() % m_TiX;
							privateBestY = rand() % m_TiY;
							privateBestZ = rand() % m_TiZ;
							
							int n = omp_get_thread_num();
							bestX[n] = privateBestX;
							bestY[n] = privateBestY;
							bestZ[n] = privateBestZ;
							mindist[n] = privateMinDist;
							
							finish_flag = true;
						}
						else if (distances < privateMinDist)
						{
							privateBestX = xTi;
							privateBestY = yTi;
							privateBestZ = zTi;
							privateMinDist = distances;
							int n = omp_get_thread_num();
							bestX[n] = privateBestX;
							bestY[n] = privateBestY;
							bestZ[n] = privateBestZ;
							mindist[n] = privateMinDist;
						}
						if(privateMinDist < m_T)
						{
							finish_flag = true;
						}
					}
				}
			}
			finish_flag = true;
			//std::cout << "search sg finished" << std::endl;
			if(finish_flag == true && assign_flag == false) {
				double bestIndex = 0;
				for(int i = 0; i < mindist.size(); i++) {
					if(mindist[i] < mindist[bestIndex])
						bestIndex = i;
				}
				//std::cout << bestZ[bestIndex] << bestX[bestIndex] << bestY[bestIndex] << std::endl;
				if (bestZ[bestIndex] == -1 || bestX[bestIndex] == -1 || bestY[bestIndex] == -1) {
					sim_value = m_Ti[rand() % m_TiZ][rand() % m_TiX][rand() % m_TiY];
				} else {
					sim_value = m_Ti[bestZ[bestIndex]][bestX[bestIndex]][bestY[bestIndex]];
				}
				assign_flag = true;
			}
			MPI_Request request;
			vector<double> sim_result;
			sim_result.push_back(sim_value);
			sim_result.push_back(sim_node_index);
			MPI_Isend(&sim_result[0], 2, MPI_DOUBLE, 0, 888, MPI_COMM_WORLD, &request);
			//MPI_Isend(&sim_node_index, 1, MPI_INT, 0, 777, MPI_COMM_WORLD, &request);
			MPI_Wait(&request, &status);
			//std::cout << current_pid << " send NO." << sim_node_index << " value " << sim_value << std::endl;
		}
		delete pathTi;
		pathTi = NULL;
	}
	IsSimulation = true;
	return "Success";
}

bool CSimulation::isConflictExists(vector<C3DPoint> &EffectivePoint, vector<int> Usim, int x, int y, int z){
	bool conflict = false;
	int maxPoints = m_SearchArea.MySearchAreaData.Sphere.MaxPoints;                                                                                                                                  
	//std::cout << "maxPOints: " << maxPoints << std::endl;
	int radius = m_SearchArea.MySearchAreaData.Sphere.SearchRadius;
	int tempX, tempY, tempZ;
	if (EffectivePoint.size() == 0){
		for(int i = 0; i< Usim.size(); i++){
			int index = Usim[i];
			PathToXYZ(index, tempX, tempY, tempZ, m_SimX, m_SimY);
			double tempDist = sqrt(double((tempX - x) * (tempX - x) + (tempY - y) * (tempY - y) + (tempZ - z) * (tempZ - z)));
			if(tempDist < radius){
				conflict = true;
				break;
			}
        }
	}else {
		int farestIndex = EffectivePoint.size() - 1;
		double d1 = EffectivePoint[farestIndex].distances;
		double d2 = TMP_MAX;
		for(int i = 0; i< Usim.size(); i++){
			int index = Usim[i];
			PathToXYZ(index, tempX, tempY, tempZ, m_SimX, m_SimY);
			double tempDist = sqrt(double((tempX - x) * (tempX - x) + (tempY - y) * (tempY - y) + (tempZ - z) * (tempZ - z)));
			if(tempDist < d2){
				d2 = tempDist;
			}
		 }
		if (EffectivePoint.size() == maxPoints){
			if (d2 < d1){
				conflict = true;
			}
		} else if (EffectivePoint.size() < maxPoints) {
			if (d2 < radius){
				conflict = true;
			}
		}
	}
	return conflict;
}

void CSimulation::ResetSim()
{
	p_PathSim.clear();
	IsSimulation = false;
	for (int i = 0; i < m_SimZ; i++)
	{
		for (int j = 0; j < m_SimX; j++)
		{
			for (int k = 0; k < m_SimY; k++)
			{
				m_Sim[i][j][k] = -1;
			}
		}
	}
}

CSimulation::~CSimulation()
{
	if (p_ScanCouts != NULL)
	{
delete p_ScanCouts;
		p_ScanCouts = NULL;
	}
	if (p_Bestmin != NULL)
	{
		delete p_Bestmin;
		p_Bestmin = NULL;
	}
}


std::string CSimulation::InsertSamples()
{
	if (m_Sim.size() < 1)
	{
		return "Error in INIT Z";
	}
	else if (m_Sim[0].size() < 1)
	{
		return "Error in INIT X";
	}
	else if (m_Sim[0][0].size() < 1)
	{
		return "Error in INIT Y";
	}

	for (int i = 0; i < m_Samples.size(); i++)
	{
		if (m_Samples[i].z < m_Sim.size() && m_Samples[i].x < m_Sim[0].size() && m_Samples[i].y < m_Sim[0][0].size())
		{
			m_Sim[m_Samples[i].z][m_Samples[i].x][m_Samples[i].y] = m_Samples[i].values;
			m_SimMark[m_Samples[i].z][m_Samples[i].x][m_Samples[i].y] = true;
		}
		else
		{
			return "sample index out of bound";
		}
	}
	return "Successfully loaded sample data!";
}

int * CSimulation::SetPath_TI(int X, int Y, int Z)
{	
	int current_pid, nameLen, comm_size;
  char processorName[MPI_MAX_PROCESSOR_NAME];
  MPI_Comm_rank(MPI_COMM_WORLD, &current_pid);
  MPI_Comm_size(MPI_COMM_WORLD, &comm_size);
  MPI_Get_processor_name(processorName, &nameLen);
  srand((unsigned)time(NULL) + comm_size * current_pid + nameLen);
//	int current_pid;
//	MPI_Comm_rank(MPI_COMM_WORLD, &current_pid);
//	srand(1000 * current_pid);

	int sizeXYZ = X * Y * Z;
	int *p_Path = new int[sizeXYZ];
	for (int i = 0; i < sizeXYZ; i++)
		p_Path[i] = i;
	for (int i = 0; i < sizeXYZ; i++)
	{
		if (i < sizeXYZ)
		{
			int m = rand() % sizeXYZ;
			//std::cout << m <<std::endl;
			int temp = p_Path[i];
			p_Path[i] = p_Path[m];
			p_Path[m] = temp;
		}
	}
	return p_Path;
}
/**
  * Changed By Zhsc 20200605
  * create pure sim path function
  */
void CSimulation::SetPath_Simul(int xRadius, int yRadius, int zRadius)
{
	// pure simpath function
	for (int z = 0; z < m_SimZ; z++) {
		for (int x = 0; x < m_SimX; x++) {
			for (int y = 0; y < m_SimY; y++) {
				int index = 0;
				if (m_Sim[z][x][y] == -1) {
					index = XYZToPath(x, y, z, m_SimX, m_SimY);
					sim_path.push_back(index);
				}
			}
		}
	}
	int current_pid, nameLen, comm_size;
	char processorName[MPI_MAX_PROCESSOR_NAME];
	MPI_Comm_rank(MPI_COMM_WORLD, &current_pid);
	MPI_Comm_size(MPI_COMM_WORLD, &comm_size);     
	MPI_Get_processor_name(processorName, &nameLen);
    srand((unsigned)time(NULL) + comm_size * current_pid + nameLen);
	
	for (int i = 0; i < sim_path.size(); i++) {
		int m = rand() % (sim_path.size());
		int temp = sim_path[i];
		sim_path[i] = sim_path[m];
		sim_path[m] = temp;
		
	}
	return;
}

int CSimulation::Compare3DPoint(C3DPoint a, C3DPoint b)
{
	if (a.distances < b.distances)
		return 1; //ÉýÐòÅÅÁÐ£¬Èç¹ûžÄÎª a >b£¬ÔòÎªœµÐò,Òª×¢Òâsort()ÖÐcmp()µÄ·µÖµÖ»ÓÐ1ºÍ0£¬²»ÏñqsortÖÐŽæÔÚ£­1£¡£¡£¡£¡
	else
		return 0;
}

int CSimulation::ComparePath(vector<int> a, vector<int> b)
{
	if (a[1] > b[1])
		return 1; //œµÐòÅÅÁÐ£¬Èç¹ûžÄÎª a <b£¬ÔòÎªÉýÐò,Òª×¢Òâsort()ÖÐcmp()µÄ·µÖµÖ»ÓÐ1ºÍ0£¬²»ÏñqsortÖÐŽæÔÚ£­1£¡£¡£¡£¡
	else
		return 0;
}

int CSimulation::CaculateSimDensity(int path, int xRadius, int yRadius, int zRadius)
{
	int Result = 0;
	int x, y, z;
	int x0, x1, y0, y1, z0, z1;
	PathToXYZ(path, x, y, z, m_SimX, m_SimY);
	x0 = x - xRadius > 0 ? x - xRadius : 0;
	x1 = x + xRadius < m_SimX - 1 ? x + xRadius : m_SimX - 1;
	y0 = y - yRadius > 0 ? y - yRadius : 0;
	y1 = y + yRadius < m_SimY - 1 ? y + yRadius : m_SimY - 1;
	z0 = z - zRadius > 0 ? z - zRadius : 0;
	z1 = z + zRadius < m_SimZ - 1 ? z + zRadius : m_SimZ - 1;
	for (int k = z0; k <= z1; k++)
	{
		for (int i = x0; i <= x1; i++)
		{
			for (int j = y0; j <= y1; j++)
			{
				if (abs(-1 - m_Sim[k][i][j]) > 1e-6)
				{
					Result++;
				}
			}
		}
	}
	return Result;
}

void CSimulation::ChangeSimDensity(int x, int y, int z, int xRadius, int yRadius, int zRadius)
{
	int x0, x1, y0, y1, z0, z1;
	x0 = x - xRadius > 0 ? x - xRadius : 0;
	x1 = x + xRadius < m_SimX ? x + xRadius : m_SimX;
	y0 = y - yRadius > 0 ? y - yRadius : 0;
	y1 = y + yRadius < m_SimY ? y + yRadius : m_SimY;
	z0 = z - zRadius > 0 ? z - zRadius : 0;
	z1 = z + zRadius < m_SimZ ? z + zRadius : m_SimZ;
	for (int k = z0; k < z1; k++)
	{
		for (int i = x0; i < x1; i++)
		{
			for (int j = y0; j < y1; j++)
			{
				int path;
				path = XYZToPath(i, j, k, m_SimX, m_SimY);
				for (int m = 0; m < p_PathSim.size(); m++)
				{
					if (p_PathSim[m][0] == path)
					{
						p_PathSim[m][1]++;
					}
				}
			}
		}
	}
}

int CSimulation::XYZToPath(int x, int y, int z, int xMax, int yMax)
{
	return z * xMax * yMax + x * yMax + y;
}

void CSimulation::PathToXYZ(int path, int & x, int & y, int & z, int xMax, int yMax)
{
	z = path / (xMax * yMax);
	int tempTi = path % (xMax * yMax);
	x = tempTi / yMax;
	y = tempTi % yMax;
}

void CSimulation::InsertEffectivePoint(vector<C3DPoint> &EffectivePoint, C3DPoint point, int nodeX, int nodeY, int nodeZ)
{
	if (m_SearchArea.MySearchAreaData.Sphere.IsUseSamePathSize == false)
	{
		if (point.x < nodeX)
		{
			_x0 = _x0 > nodeX - point.x ? _x0 : nodeX - point.x;
		}
		else
		{
			_x1 = _x1 > point.x - nodeX ? _x1 : point.x - nodeX;
		}

		if (point.y < nodeY)
		{
			_y0 = _y0 > nodeY - point.y ? _y0 : nodeY - point.y;
		}
		else
		{
			_y1 = _y1 > point.y - nodeY ? _y1 : point.y - nodeY;
		}

		if (point.z < nodeZ)
		{
			_z0 = _z0 > nodeZ - point.z ? _z0 : nodeZ - point.z;
		}
		else
		{
			_z1 = _z1 > point.z - nodeZ ? _z1 : point.z - nodeZ;
		}
	}
	EffectivePoint.push_back(point);
}

void CSimulation::GetEffectivePoint(vector<C3DPoint>& EffectivePoint, int nodeX, int nodeY, int nodeZ)
{
	int z0x0y0, z0x1y0;
	int z0x0y1, z0x1y1;
	int z1x0y0, z1x1y0;
	int z1x0y1, z1x1y1;
	vector<C3DPoint> temp;					//·¶Î§ÄÚÒÑÖªµã£¬²»ÊÇÑùÆ·µã(ÌõŒþÊýŸÝÓÅÏÈÊ±£¬²»ÓÅÏÈŸÍÊÇËùÓÐµã)
	vector<C3DPoint> tempSample;			//·¶Î§ÄÚÒÑÖªµã£¬ÊÇÑùÆ·µã

	z0x0y0 = z0x1y0 = z0x0y1 = z0x1y1 = z1x0y0 = z1x1y0 = z1x0y1 = z1x1y1 = 0;

	int xRadius = 0, yRadius = 0, zRadius = 0;

	switch (m_SearchArea.MySearchAreaType)
	{
	case SearchArea::SearchAreaType::Cuboid:
		xRadius = m_SearchArea.MySearchAreaData.Cuboid.HalfX;
		yRadius = m_SearchArea.MySearchAreaData.Cuboid.HalfY;
		zRadius = m_SearchArea.MySearchAreaData.Cuboid.HalfZ;
		break;
	case SearchArea::SearchAreaType::Sphere:
		xRadius = m_SearchArea.MySearchAreaData.Sphere.SearchRadius;
		yRadius = m_SearchArea.MySearchAreaData.Sphere.SearchRadius;
		zRadius = m_SearchArea.MySearchAreaData.Sphere.SearchRadius;
		break;
	default:
		break;
	}

	int x0 = 0 > nodeX - xRadius ? 0 : nodeX - xRadius;						//XÆðÊŒ
	int x1 = m_SimX - 1 < nodeX + xRadius ? m_SimX - 1 : nodeX + xRadius;
	int y0 = 0 > nodeY - yRadius ? 0 : nodeY - yRadius;						//YÆðÊŒ
	int y1 = m_SimY - 1 < nodeY + yRadius ? m_SimY - 1 : nodeY + yRadius;
	int z0 = 0 > nodeZ - zRadius ? 0 : nodeZ - zRadius;						//ZÆðÊŒ
	int z1 = m_SimZ - 1 < nodeZ + zRadius ? m_SimZ - 1 : nodeZ + zRadius;

	/**
	* 2020/05/10 
	* ZHSC
	* ÐÂÔöŽÓÄÚÏòÍâ²éÕÒÓÐÐ§µã£š±éÀú¡¢ÅÐœç¡¢ÅÐÖØ¡¢ÓÐÐ§µãžöÊýŽïµœºóÖ±œÓ·µ»Ø£©
	* ÐèÒªÊ¹ÓÃÔ­²éÕÒÓÐÐ§µã·œ·šÊ±ÐèÈ«²¿×¢ÊÍµô
	* µ±Ç°ŽæÔÚÎÊÌâ£ºÓÐÐ§µã³€¶È²»Îª0Ê±ÎªºÎdistanceÎª-1
	**/
	//std::cout << "get effective point DEBUG: " << nodeX << " " << nodeY << " " << nodeZ << std::endl;	
	//¶þÎ¬ÍŒÏñ
	if (z0 == z1)
	{
		if (m_SearchArea.MySearchAreaType == SearchArea::SearchAreaType::Sphere) {
			int cnt = m_SearchArea.MySearchAreaData.Sphere.MaxPoints;
			int radius = m_SearchArea.MySearchAreaData.Sphere.SearchRadius;
			
			//Ã¿ŽÎÀ©Õ¹µÄžñ×ÓÊýÎª1
			for (size_t m = 1; m <= radius; m++)
			{
				
				int xmin = nodeX - m;
				int xmax = nodeX + m;
				int ymin = nodeY - m;
				int ymax = nodeY + m;
				for (size_t iX = xmin; iX <= xmax; iX++)
				{
					if (iX >= x0 && iX <= x1)
					{
						if (ymin >= y0)
						{
							double distances = sqrt(double((iX - nodeX) * (iX - nodeX) + (ymin - nodeY) * (ymin - nodeY)));
							if (distances <= m_SearchArea.MySearchAreaData.Sphere.SearchRadius && m_Sim[z0][iX][ymin] != -1)
							{
								InsertEffectivePoint(EffectivePoint, C3DPoint(iX, ymin, z0, m_Sim[z0][iX][ymin], distances), nodeX, nodeY, nodeZ);
								//EffectivePoint.push_back(C3DPoint(iX, ymin, z0, m_Sim[z0][iX][ymin], distances));
							}
							if (EffectivePoint.size() == cnt)
							{
								return;
							}
						}
						if (ymax <= y1)
						{
							double distances = sqrt(double((iX - nodeX) * (iX - nodeX) + (ymax - nodeY) * (ymax - nodeY)));
							if (distances <= m_SearchArea.MySearchAreaData.Sphere.SearchRadius && m_Sim[z0][iX][ymax] != -1)
								InsertEffectivePoint(EffectivePoint, C3DPoint(iX, ymax, z0, m_Sim[z0][iX][ymax], distances), nodeX, nodeY, nodeZ);
								//EffectivePoint.push_back(C3DPoint(iX, ymax, z0, m_Sim[z0][iX][ymax], distances));
							if (EffectivePoint.size() == cnt)
							{
								return;
							}
						}
					}
					
				}
				for (size_t iY = ymin + 1; iY <= ymax - 1; iY++)
				{
					if (iY >= y0 && iY <= y1)
					{
						if (xmin >= x0)
						{
							double distances = sqrt(double((xmin - nodeX) * (xmin - nodeX) + (iY - nodeY) * (iY - nodeY)));
							if (distances <= m_SearchArea.MySearchAreaData.Sphere.SearchRadius && m_Sim[z0][xmin][iY] != -1)
								InsertEffectivePoint(EffectivePoint, C3DPoint(xmin, iY, z0, m_Sim[z0][xmin][iY], distances), nodeX, nodeY, nodeZ);
								//EffectivePoint.push_back(C3DPoint(xmin, iY, z0, m_Sim[z0][xmin][iY], distances));
							if (EffectivePoint.size() == cnt)
							{
								return;
							}
						}
						if (xmax <= x1)
						{
							
							double distances = sqrt(double((xmax - nodeX) * (xmax - nodeX) + (iY - nodeY) * (iY - nodeY)));
							if (distances <= m_SearchArea.MySearchAreaData.Sphere.SearchRadius && m_Sim[z0][xmax][iY] != -1)
								InsertEffectivePoint(EffectivePoint, C3DPoint(xmax, iY, z0, m_Sim[z0][xmax][iY], distances), nodeX, nodeY, nodeZ);
								//EffectivePoint.push_back(C3DPoint(xmax, iY, z0, m_Sim[z0][xmax][iY], distances));
							if (EffectivePoint.size() == cnt)
							{
								return;
							}
						}
					}
				}
			}
		
		}
	}
	else //ÈýÎ¬ÍŒÏñ
	{
		if (m_SearchArea.MySearchAreaType == SearchArea::SearchAreaType::Sphere) {
			int cnt = m_SearchArea.MySearchAreaData.Sphere.MaxPoints;
			int radius = m_SearchArea.MySearchAreaData.Sphere.SearchRadius;
			for (int m = 1; m <= radius; m++)
			{
				int xmin = nodeX - m;
				int xmax = nodeX + m;
				int ymin = nodeY - m;
				int ymax = nodeY + m;
				int zmin = nodeZ - m;
				int zmax = nodeZ + m;
				for (int iX = nodeX - m; iX <= nodeX + m; iX++)
				{
					if (iX >= x0 && iX <= x1)
					{
						for (int iY = nodeY - m; iY <= nodeY + m; iY++)
						{
							if (iY >= y0 && iY <= y1)
							{
								if (zmin >= z0)
								{
									double distances = sqrt(double((iX - nodeX) * (iX - nodeX) + (iY - nodeY) * (iY - nodeY) + (zmin - nodeZ) * (zmin - nodeZ)));
									if (distances <= m_SearchArea.MySearchAreaData.Sphere.SearchRadius && m_Sim[zmin][iX][iY] != -1)
										InsertEffectivePoint(EffectivePoint, C3DPoint(iX, iY, zmin, m_Sim[zmin][iX][iY], distances), nodeX, nodeY, nodeZ);
										//EffectivePoint.push_back(C3DPoint(iX, iY, zmin, m_Sim[zmin][iX][iY], distances));
									if (EffectivePoint.size() == cnt)
									{
										return;
									}
								}
								if (zmax <= z1)
								{
									double distances = sqrt(double((iX - nodeX) * (iX - nodeX) + (iY - nodeY) * (iY - nodeY) + (zmax - nodeZ) * (zmax - nodeZ)));
									if (distances <= m_SearchArea.MySearchAreaData.Sphere.SearchRadius && m_Sim[zmax][iX][iY] != -1)
										InsertEffectivePoint(EffectivePoint, C3DPoint(iX, iY, zmax, m_Sim[zmax][iX][iY], distances), nodeX, nodeY, nodeZ);
										//EffectivePoint.push_back(C3DPoint(iX, iY, zmax, m_Sim[zmax][iX][iY], distances));
									if (EffectivePoint.size() == cnt)
									{
										return;
									}
								}
							}
						}
					}
				} 
				//×óÓÒÁœžöÃæ
				for (int iY = nodeY - m; iY <= nodeY + m; iY++)
				{
					if (iY >= y0 && iY <= y1) //±ßœçµãÅÐ¶Ï
					{
						//Áœ¶ËÈ¥ÖØ
						for (int iZ = nodeZ - m + 1; iZ < nodeZ + m; iZ++)
						{
							if (iZ >= z0 && iZ <= z1)
							{
								if (xmin >= x0)
								{
									double distances = sqrt(double((xmin - nodeX) * (xmin - nodeX) + (iY - nodeY) * (iY - nodeY) + (iZ - nodeZ) * (iZ - nodeZ)));
									if (distances <= m_SearchArea.MySearchAreaData.Sphere.SearchRadius && m_Sim[iZ][xmin][iY] != -1)
										InsertEffectivePoint(EffectivePoint, C3DPoint(xmin, iY, iZ, m_Sim[iZ][xmin][iY], distances), nodeX, nodeY, nodeZ);
										//EffectivePoint.push_back(C3DPoint(xmin, iY, iZ, m_Sim[iZ][xmin][iY], distances));
									if (EffectivePoint.size() == cnt)
									{
										return;
									}
								}
								if (xmax <= x1)
								{
									double distances = sqrt(double((xmax - nodeX) * (xmax - nodeX) + (iY - nodeY) * (iY - nodeY) + (iZ - nodeZ) * (iZ - nodeZ)));
									if (distances <= m_SearchArea.MySearchAreaData.Sphere.SearchRadius && m_Sim[iZ][xmax][iY] != -1)
										InsertEffectivePoint(EffectivePoint, C3DPoint(xmax, iY, iZ, m_Sim[iZ][xmax][iY], distances), nodeX, nodeY, nodeZ);
										//EffectivePoint.push_back(C3DPoint(xmax, iY, iZ, m_Sim[iZ][xmax][iY], distances));
									if (EffectivePoint.size() == cnt)
									{
										return;
									}
								}
							}
						}
					}
				}
				//Ç°ºóÁœžöÃæ
				for (int iX = nodeX - m + 1; iX < nodeX + m; iX++)
				{
					if (iX >= x0 && iX <= x1)
					{
						for (int iZ = nodeZ - m + 1; iZ < nodeZ + m; iZ++) {
							if (iZ >= z0 && iZ <= z1)
							{
								if (ymin >= y0) 
								{
									double distances = sqrt(double((iX - nodeX) * (iX - nodeX) + (ymin - nodeY) * (ymin - nodeY) + (iZ - nodeZ) * (iZ - nodeZ)));
									if (distances <= m_SearchArea.MySearchAreaData.Sphere.SearchRadius && m_Sim[iZ][iX][ymin] != -1)
										InsertEffectivePoint(EffectivePoint, C3DPoint(iX, ymin, iZ, m_Sim[iZ][iX][ymin], distances), nodeX, nodeY, nodeZ);
										//EffectivePoint.push_back(C3DPoint(iX, ymin, iZ, m_Sim[iZ][iX][ymin], distances));
									if (EffectivePoint.size() == cnt)
									{
										return;
									}
								}
								if (ymax <= y1)
								{
									double distances = sqrt(double((iX - nodeX) * (iX - nodeX) + (ymax - nodeY) * (ymax - nodeY) + (iZ - nodeZ) * (iZ - nodeZ)));
									if (distances <= m_SearchArea.MySearchAreaData.Sphere.SearchRadius && m_Sim[iZ][iX][ymax] != -1)
										InsertEffectivePoint(EffectivePoint, C3DPoint(iX, y1, iZ, m_Sim[iZ][iX][ymax], distances), nodeX, nodeY, nodeZ);
										//EffectivePoint.push_back(C3DPoint(iX, y1, iZ, m_Sim[iZ][iX][ymax], distances));
									if (EffectivePoint.size() == cnt)
									{
										return;
									}
								}
							}
						}
					}
				}
			}
		}
	}

	return;
}

//TODO BUG: GetDistances erase all the node in EffectivePoint
double CSimulation::GetDistances(vector<C3DPoint> EffectivePoint, int x, int y, int z, int nodeX, int nodeY, int nodeZ)
{
	double sum = 0;
	for (vector<C3DPoint>::iterator it = EffectivePoint.begin(); it != EffectivePoint.end(); )
	{
		//µ±ÑµÁ·ÍŒÏñÎª¶þÎ¬£¬ÒªÄ£ÄâÈýÎ¬£¬³öÎÊÌâ£¬ŒÓÒÔÏÂÅÅ³ý£¬ÕâÐ©µã²»ÄÜËãŸàÀë
		if (z - nodeZ + it->z < 0 || z - nodeZ + it->z >= m_TiZ || x - nodeX + it->x < 0 || x - nodeX + it->x >= m_TiX || y - nodeY + it->y < 0 || y - nodeY + it->y >= m_TiY)
		{
			//std::cout << "GetDistances erase EffectivePoint" <<std::endl;
			std::cout << nodeX << " " << nodeY << " " << nodeZ << std::endl;
			it = EffectivePoint.erase(it);
			continue;
		}

		switch (m_DistanceType)
		{
		case Different:
			if (abs(it->values - m_Ti[z - nodeZ + it->z][x - nodeX + it->x][y - nodeY + it->y]) > 1e-6)
			{
				sum += 1.0;
			}
			//std::cout << "different type" << std::endl;
			break;
		case Variance:
			sum += (it->values - m_Ti[z - nodeZ + it->z][x - nodeX + it->x][y - nodeY + it->y])*(it->values - m_Ti[z - nodeZ + it->z][x - nodeX + it->x][y - nodeY + it->y]);
			break;
		case WeightedVariance:
			sum += (it->values - m_Ti[z - nodeZ + it->z][x - nodeX + it->x][y - nodeY + it->y])*(it->values - m_Ti[z - nodeZ + it->z][x - nodeX + it->x][y - nodeY + it->y]);
			break;
		default:
			break;
		}
		it++;
	}
	if (EffectivePoint.size() == 0)
	{
		return -1;
	}
	return sum / EffectivePoint.size();
	
}
