#include "Simulation.h"
#include "mpi.h"

static string paraFile = "params.txt";

string getSavePath(string tiPath, string s_num);

int main(int argc, char** argv){

	MPI_Init(&argc, &argv);
	//std::cout << "time: " << MPI_Wtime() << std::endl;
	int pid, psize;
	MPI_Comm_rank(MPI_COMM_WORLD, &pid);
	MPI_Comm_size(MPI_COMM_WORLD, &psize);
	MPI_Status status;

	CSimulation* p_Simulation;

	//init parameters
	int m_Simul_Num = 1; //number of multi simulate
	bool m_IsHaveSample = true; //if have sample data
	int m_SimX;
	int m_SimY;
	int m_SimZ;
	float m_Fract;
	float m_Thr;
	int m_MaxPoints;
	int m_SearchRadius;
	string tiPath = "";
	string samplesPath = "";
	if (pid == 0) { 
		//read parameters from param file
		stringstream buffer;
		string line;
		std::vector<string> params;
		ifstream fin;
		fin.open(paraFile);
		if (!fin) {
			std::cout << "param file not found!" << std::endl;
			return -1;
		}
		while (getline(fin, line)) {
			if (line[0] != '#' && !(line.compare("") == 0)){
				params.push_back(line);
			}
		}
		//convert string to value
		m_Simul_Num = atoi(params[0].c_str());
		string str_index[3];
		istringstream is(params[1]);
		is>>str_index[0]>>str_index[1]>>str_index[2];
		m_SimX = atoi(str_index[0].c_str());
		m_SimY = atoi(str_index[1].c_str());
		m_SimZ = atoi(str_index[2].c_str());
		m_Fract = atof(params[2].c_str());
		m_Thr = atof(params[3].c_str());
		m_MaxPoints = atoi(params[4].c_str());
		m_SearchRadius = atoi(params[5].c_str());
		tiPath.append(params[7]);
		samplesPath.append(params[8]);
		for(int ii = 1; ii < psize; ii++){
			vector<double> algorithmPara;
			algorithmPara.push_back(m_SimX);
			algorithmPara.push_back(m_SimY);
			algorithmPara.push_back(m_SimZ);
			algorithmPara.push_back(m_Fract);
			algorithmPara.push_back(m_Thr);
			algorithmPara.push_back(m_MaxPoints);
			algorithmPara.push_back(m_SearchRadius);
			int size = algorithmPara.size();
			MPI_Send(&size, 1, MPI_INT, ii, 1, MPI_COMM_WORLD);
			MPI_Send(&algorithmPara[0], size, MPI_DOUBLE, ii, 11, MPI_COMM_WORLD);
		}	
	} else {
		int buffer_size;
		vector<double> buffer_paras;
		MPI_Recv(&buffer_size, 1, MPI_INT, 0, 1, MPI_COMM_WORLD, &status);
		buffer_paras.resize(buffer_size);
		MPI_Recv(&buffer_paras[0], buffer_size, MPI_DOUBLE, 0, 11, MPI_COMM_WORLD, &status);
		m_SimX = (int) buffer_paras[0];
		m_SimY = (int) buffer_paras[1];
		m_SimZ = (int) buffer_paras[2];
		m_Fract = buffer_paras[3];
		m_Thr = buffer_paras[4];
		m_MaxPoints = (int) buffer_paras[5];
		m_SearchRadius = (int) buffer_paras[6];
	}

	SearchArea searchArea;
		searchArea.MySearchAreaType = SearchArea::SearchAreaType::Sphere;
		searchArea.MySearchAreaData.Sphere.MaxPoints = m_MaxPoints;
		searchArea.MySearchAreaData.Sphere.SearchRadius = m_SearchRadius;
		searchArea.MySearchAreaData.Sphere.IsUseSamePathSize = false;
		searchArea.MySearchAreaData.Sphere.IsSampleFirst = false;
	

	std::cout << "Finish load parameters." << std::endl;

	//construct
	p_Simulation = new CSimulation(m_SimX, m_SimY, m_SimZ, m_Fract, m_Thr, DistanceType::Different, searchArea, SimulatingPathType::Random);
	/*p_Simulation->m_TiX = 250;
	p_Simulation->m_TiY = 250;
	p_Simulation->m_TiZ = 1;*/

	//load ti and sample
	p_Simulation->LoadTi(tiPath);
	p_Simulation->LoadSamples(samplesPath);
	//p_Simulation->LoadTi("TI_channel2000.sgems");
	//p_Simulation->LoadSamples("channel_100.SGEMS");
	//p_Simulation->LoadTi("Berea_400.dat");
	//p_Simulation->LoadTi("fold180.SGEMS");
	//p_Simulation->LoadSamples("xyz543.sgems");
	//p_Simulation->LoadSamples("channel_10000.SGEMS");
	//p_Simulation->LoadSamples("xyz11.sgems");
	/*if (pid == 0) {
		//p_Simulation->LoadTi(tiPath);
		p_Simulation->LoadTi("TI_channel.SGEMS");
		//p_Simulation->LoadTi("TI_fold.sgems");
		//p_Simulation->LoadSamples(samplesPath);	
		p_Simulation->LoadSamples("channel_100.SGEMS");
		//p_Simulation->LoadSamples("fold_13.sgmes");
	}*/
	//p_Simulation->LoadSamples(samplesPath);
	
	/*if (!p_Simulation->IsHaveTi){
		std::cout<<"No Ti"<<std::endl;
		return -1;
	}
	if (!p_Simulation->IsHaveSamples && m_IsHaveSample){
		std::cout<<"No Samples"<<std::endl;
		return -2;
	}*/
	//p_Simulation->LoadSamples(samplesPath);
	
	clock_t startTime,endTime;
	struct timespec time1 = {0, 0};
	struct timespec time2 = {0, 0};
	clock_gettime(CLOCK_REALTIME, &time1);
	for (int i = 0; i < m_Simul_Num; i++){	
		//initialize SG
		if(pid == 0){
			p_Simulation->ResetSim();
			//startTime = clock();
		}

		std::cout << "process " << pid <<" start simulation" << std::endl;
		//start simulation
		string results = p_Simulation->StartSimulation();
		std::cout << "number: " << i << " " << results << std::endl;
		clock_gettime(CLOCK_REALTIME, &time2);
		//save realization
		stringstream s_num;
		s_num << i;
		if(pid == 0){
			//endTime = clock();
			string savePath = getSavePath(tiPath, s_num.str());
			p_Simulation->SaveSimulation(savePath, PropertyFileType::SGEMS);
			std::cout << "save success" << std::endl;
			//endTime = clock();
			std::cout << "The runtime: " << (time2.tv_sec-time1.tv_sec)*1000 + (time2.tv_nsec-time1.tv_nsec)/1000000 << "ms" << std::endl;
		}
	}
	MPI_Finalize();
	return 0;
}

string getSavePath(string tiPath, string s_num){
	//get the file name
	int l = tiPath.length();
	int i = l - 1;
	int mark = l - 1;
	for(i = l - 1; i >= 0; i--){
		if(tiPath[i] == '.') mark = i;
		if(tiPath[i] == '/') break; 
	}   
	string filename = tiPath.substr(i + 1,mark-i-1);
	string savePath = "";
	savePath.append(filename);
	savePath.append("_simulation_");
	savePath.append(s_num);
	savePath.append(".sgems");
	return savePath;
}

