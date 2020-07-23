#include<string>
#include<vector>
#include<map>
#include<set>
#include<iostream>
#include<fstream>
#include<sstream>
#include<stdlib.h>
#include<queue>
#include<cstdlib>
#include<algorithm>
#include<ctime>
#include<iomanip>
#include<unistd.h>
using namespace std;

#define DEBUG(m) {cerr<<m<<endl;exit(1);}



class Station;
class Road;
class Cargo;
class Truck;
struct Merge_cargo;
struct Merge_route;
#define STATION_BEGIN 0
#define ROAD_BEGIN 1
#define CARGO_BEGIN 1
//站点名称，站点字典
map<int,Station> NAME_STATION;
//道路名称，道路 字典
map<int,Road> NAME_ROAD;
//货物名称，货物 字典
map<int,Cargo> NAME_CARGO;
//合并操作，其名为起点终点，对应合并的货物
map<pair<int,int>,Merge_cargo> NAME_MERGE_CARGO;
//合并操作，其名位整个路径，对应合并的货物
map<vector<int>,Merge_route> NAME_MERGE_ROUTE;
//从文档种提取的第一行信息
int STATION_NUMBERS;
int ROAD_NUMBERS;
int CARGO_NUMBERS;
int CARS_PER_ROAD;
int LOAD_PER_CAR;
//用于优化，重复起点复用
map<int,pair<vector<double>,vector<int>>> BEGIN_ROUTE;

//货车类
class Truck{
public:
    //最大载重
    int maxload;
    //编号
    int No;
    //当前载重
    double cur_weight;
    //车内货物
    vector<int> cargos;
    //是否有分拣员辅助
    bool has_worker = false;
    Truck(int m,int N):maxload(m),No(N){
        cur_weight = 0;
    }
    Truck(){}
};


class Road;
//站点类
class Station{
public:
    //站名
    int name;
    //分拣工数目
    int workers;
    //连接的道路(为便于查找，设置为set),对应的站点
    set<int> roads;
    //连接的站点对应的道路,以及道路上是否有工人
    map<int,int> station_bridge;
    //道路对应的车，是否有车
    map<int,vector<bool>> road_car;
    //闲置工的个数
    int left_workers;
    //道路对应的车，是否有工人
    map<int,vector<bool>> road_car_worker;
public:
    Station(int s,int w):name(s),workers(w){
        left_workers = workers;
    }
    //
    Station(){}
};

//道路类
class Road{
public:
    //路名
    int name;
    //站1
    int station1;
    //站2
    int station2;
    //货车
    vector<Truck> trucks;
    //权值
    double k = 1.0;
    //还剩多少车可以用
    int left_car;
    
public:
    Road(int n,int s1,int s2):name(n),station1(s1),station2(s2){
        for(int i = 0;i<CARS_PER_ROAD;i++)
        {
            trucks.push_back(Truck(100,i+1));
        }
        left_car = CARS_PER_ROAD;
    }
    Road(){}

};



//货物类
class Cargo{
public:
    //货物编号
    int No;
    //起始城市
    int begin;
    //终点城市
    int end;
    //重量
    double weight;
    //必经站点
    vector<int> passby_stations;
    //路径，站点
    vector<int> path;
    //经过的道路
    vector<int> passby_roads;
    //使用的货车编号
    vector<int> truck_numbers;
    //是否成功抵达
    bool is_success = true;
    //路径的总权重
    double val = 0.0;
public:
    Cargo(int N,int b,int e,double w,vector<int> ps)
    :No(N),begin(b),end(e),weight(w),passby_stations(ps){}
    Cargo(){};
    //货物内部的不汇聚、不拆分、不换车厢，没有必经站点的dijkstra,所有权值都设置为1
    void dijkstra()
    {
        int size = NAME_STATION.size();
        vector<bool> vis(size,false);
        
        //用于记录某节点的前结点
        vector<int> pre(size);
        //dis表示cargo的位置到某一节点的位置
        vector<double> dis(size,1e8*1.0); 

        //一个结点的信息,pq，默认最大堆，这里与它反着搞
        struct Node{
            double dis;
            int pre_road_id;
            int pos;
            bool operator<(const Node & r) const{return r.dis < dis;}
        };

        

        priority_queue<Node> pq;
        dis[begin] = 0;
        pq.push({0.0,-1,begin});
        pre[begin] = -1;

        

        while(!pq.empty())
        {
            Node cur_node = pq.top();
            pq.pop();
            if(vis[cur_node.pos])
                continue;
            vis[cur_node.pos] = true;
            
            //遍历临接站点，找最小临接矩阵
            Station station = NAME_STATION[cur_node.pos];
            set<int> & roads = station.roads;
            if(roads.empty())
                DEBUG("station.road is empty!");
            for(auto iter = roads.begin();iter!=roads.end();iter++)
            {
                Road road = NAME_ROAD[*iter];
                //该公路的另一端
                int another_station = (road.station1 == station.name?road.station2:
                    road.station1);
                //如果该点没有被遍历过
                //测试
                
                if(!vis[another_station])
                {
                    //cout << "here" <<endl;
                    int pos = cur_node.pos;
                    int as = another_station;
                    double val = road.k;
                    if(dis[pos]+val<dis[as])
                    {
                        dis[as] = dis[pos]+val;
                        pq.push({dis[as],pos,as});
                        //测试
                        pre[as] = pos;
                    }
                }
            }
        }

        
        
        //输出路径之前校验是否可达
        if(dis[end]>=10000000)
        {
            //cout << "Cargo "<<No <<" can't reach end\n";
            return;
        }
        //BEGIN_ROUTE[begin] = {dis,pre};
        //输出路径
        val = dis[end];
        int i = end;
        while(i!=-1)
        {
            path.push_back(i);
            i = pre[i];
        }
        reverse(path.begin(),path.end());
        
    }
    friend bool operator<(const Cargo & c1,const Cargo & c2)
    {
        return c1.weight<=c2.weight;
    }
    friend bool operator>(const Cargo & c1,const Cargo & c2)
    {
        return c1.weight>c2.weight;
    }
    
};

//普通货物的源宿合并结构体
struct Merge_cargo
{
    //int name;
    //合并的货物id，依重量从小到大排序
    //第一个为重量，第二个为id
    set<pair<double,int>> cargos;
    //起点
    int begin;
    int end;
    Merge_cargo(int b,int e):begin(b),end(e){}
    void push_back(int cargo_id)
    {
        cargos.insert({NAME_CARGO[cargo_id].weight,cargo_id});
    }
    //找到一定区间内的货物，并按照重量从小到大顺序返回
    vector<int> find_weight(double max_weight,int num)
    { 
        vector<int> v_cargo_id;
        int index = 0;
        for(auto iter = cargos.begin();iter!=cargos.end()&&index<num;iter++,index++)
        {
            if(iter->first<=max_weight)
            {
                v_cargo_id.push_back(iter->second);
            }
            else
                break;
        }
        return v_cargo_id;
    }
    //删除操作
    void erase(int cargo_id)
    {
        cargos.erase({NAME_CARGO[cargo_id].weight,cargo_id});
    }
    //生成一份货运，用于派发到货车上，派发使用贪心的思想，当前最大重量货物与若干最小货物合并。
    vector<int> create_pack(double & cur_load)
    {
        auto cur_max_weight_cargo = --cargos.end();
        auto cur_min_weight_cargo = cargos.begin();
        
        
        if(cargos.size()==0)
            DEBUG("cargow.size()?")

        double max_load = LOAD_PER_CAR;
        cur_load = 0;

        vector<int> result;
        while(cur_load<=max_load)
        {
            double max_weight = cur_max_weight_cargo->first;
            double min_weight = cur_min_weight_cargo->first;
            int max_cargo_id = cur_max_weight_cargo->second;
            int min_cargo_id = cur_min_weight_cargo->second;
            if(cur_load+max_weight<=max_load)
            {
                cur_load+=max_weight;
                cargos.erase(*cur_max_weight_cargo);
                result.push_back(max_cargo_id);
                if(cur_max_weight_cargo == cur_min_weight_cargo)
                    break;
                cur_max_weight_cargo = --cargos.end();
                continue;
            }
            if(cur_load+min_weight<=max_load)
            {
                cur_load+=min_weight;
                cargos.erase(*cur_min_weight_cargo);
                result.push_back(min_cargo_id);
                if(cur_max_weight_cargo == cur_min_weight_cargo)
                    break;
                cur_min_weight_cargo = cargos.begin();
                continue;
            }
            else break;
        }
        return result;
    }
    Merge_cargo(){}
};

//特殊货物同路径源宿合并合并
struct Merge_route
{
    //第一个为重量，第二个为id
    set<pair<double,int>> cargos;
    vector<int> route;
    Merge_route(vector<int> r):route(r){}
    Merge_route(){}
    //压入操作
    void push_back(int cargo_id)
    {
        cargos.insert({NAME_CARGO[cargo_id].weight,cargo_id});
    }
     //删除操作
    void erase(int cargo_id)
    {
        cargos.erase({NAME_CARGO[cargo_id].weight,cargo_id});
    }
    //该合并原理同普通货物合并原理
    vector<int> create_pack(double & cur_load)
    {
        auto cur_max_weight_cargo = --cargos.end();
        auto cur_min_weight_cargo = cargos.begin();
        
        // cout << cur_max_weight_cargo->first << "   "<<cur_max_weight_cargo->second<<endl;
        // cout << cur_min_weight_cargo->first<<"   "<<cur_min_weight_cargo->second<<endl;
        
        if(cargos.size()==0)
            DEBUG("cargow.size()?")

        double max_load = LOAD_PER_CAR;
        cur_load = 0;

        vector<int> result;
        while(cur_load<=max_load)
        {
            //测试
            // cout << "in loop"<<endl;
            // cout << "cur load  "<<cur_load<<"  max_load  "<<max_load<<endl;
            //sleep(1);
            double max_weight = cur_max_weight_cargo->first;
            double min_weight = cur_min_weight_cargo->first;
            int max_cargo_id = cur_max_weight_cargo->second;
            int min_cargo_id = cur_min_weight_cargo->second;
            if(cur_load+max_weight<=max_load)
            {
                cur_load+=max_weight;
                cargos.erase(*cur_max_weight_cargo);
                result.push_back(max_cargo_id);
                if(cur_max_weight_cargo == cur_min_weight_cargo)
                    break;
                cur_max_weight_cargo = --cargos.end();
                continue;
            }
            if(cur_load+min_weight<=max_load)
            {
                cur_load+=min_weight;
                cargos.erase(*cur_min_weight_cargo);
                result.push_back(min_cargo_id);
                if(cur_max_weight_cargo == cur_min_weight_cargo)
                    break;
                cur_min_weight_cargo = cargos.begin();
                continue;
            }
            else break;
        }
        return result;
    }
};

//执行类
class Solution
{
public:
    //输入文件名
    string file_name;
    //失败货物个数
    int fail_cargo = 0;
    //失败货物的总重量
    double total_weight = 0.0;
    //用作计数
    int times = 0;
    //需要经过固定点的车
    vector<int> need_pass_by_staion;
    
public:
    //执行入口
    Solution(string f_n):file_name(f_n)
    {
        //数据加载
        this->load_file_result();
        //处理数据
        this->start_load();
        //输出数据
        this->start_run_result();
    }
    //加载数据并构图(成功)
    void load_file()
    {
        //将ROAD和CARGO添加一个空的头，这样其编号和vector的下标对应起来了
        
        fstream f(file_name);
        if(!f.is_open())
            cerr<<"file is not open\n";
        string line;
        bool flag = false;
        while(getline(f,line))
        {
            for(int i = 0;i<line.size();i++)
            {
                if(line[i] == ',')
                    line[i] = ' ';
            }
            if(!flag)
            {
                flag = true;
                stringstream sstream(line);
                
                string tmp;
                sstream>>tmp;
                STATION_NUMBERS = atoi(tmp.c_str());
                sstream>>tmp;
                ROAD_NUMBERS = atoi(tmp.c_str());
                sstream>>tmp;
                CARS_PER_ROAD = atof(tmp.c_str());
                sstream>>tmp;
                LOAD_PER_CAR = atoi(tmp.c_str());
                continue;
            }
            //将得到的行进行处理
            
            switch(line[0])
            {
                case 'Z':
                {
                    stringstream sstream(line);
                    string name;
                    sstream>>name;
                    if(name.empty())
                        DEBUG("Z name");
                    name = name.substr(1,name.size()-1);
                    int n = atoi(name.c_str());

                    string workers;
                    sstream>>workers;
                    Station station(n,atoi(workers.c_str()));
                    NAME_STATION.insert({n,station});
                }
                break;
                case 'R':
                {
                    stringstream sstream(line);
                    string name;
                    sstream>>name;
                    if(name.empty())
                        DEBUG("R name");
                    name = name.substr(1,name.size()-1);
                    int n = atoi(name.c_str());

                    string station1;
                    sstream>>station1;
                    if(station1.empty())
                        DEBUG("R station1");
                    station1 = station1.substr(1,station1.size()-1);
                    int s1 = atoi(station1.c_str());

                   
                    string station2;
                    sstream>>station2;
                    if(station2.empty())
                        DEBUG("R station2");
                    station2 = station2.substr(1,station2.size()-1);
                    int s2 = atoi(station2.c_str());
                    Road road(n,s1,s2);
                    //站点构图
                    NAME_STATION[s1].station_bridge[s2]=n;
                    NAME_STATION[s2].station_bridge[s1]=n;
                    NAME_STATION[s1].roads.insert(n);
                    NAME_STATION[s2].roads.insert(n);
                    NAME_STATION[s1].road_car[n] = vector<bool>(CARS_PER_ROAD,false);
                    NAME_STATION[s2].road_car[n] = vector<bool>(CARS_PER_ROAD,false);
                    NAME_STATION[s1].road_car_worker[n] = vector<bool>(CARS_PER_ROAD,false);
                    NAME_STATION[s2].road_car_worker[n] = vector<bool>(CARS_PER_ROAD,false);
                    //收入NAME_ROAD
                    NAME_ROAD[n] = road;
                }
                break;
                case 'G':
                {
                    stringstream sstream(line);
                    string name;
                    sstream >> name;
                    if(name.empty())
                        DEBUG("G name");
                    name = name.substr(1,name.size()-1);
                    int n = atoi(name.c_str());
                    //测试
                    //cout<<"Cargo "<<n<<"has been put"<<endl;
                    string begin;
                    sstream >> begin;
                    if(begin.empty())
                        DEBUG("G begin");
                    begin = begin.substr(1,begin.size()-1);
                    int b = atoi(begin.c_str());

                    string end;
                    sstream >> end;
                    if(end.empty())
                        DEBUG("G end");
                    end = end.substr(1,end.size()-1);
                    int e = atoi(end.c_str());

                    string weight;
                    sstream >> weight;
                   

                    vector<int> passby_stations;
                    string tmp;
                    while(sstream>>tmp)
                    {
                        if(tmp.empty())
                            DEBUG("G tmp");
                        if(tmp == "null")
                            break;
                        tmp = tmp.substr(1,tmp.size()-1);
                        int t = atoi(tmp.c_str());
                        
                        passby_stations.push_back(t);
                    }
                    double w = atof(weight.c_str());
                    //测试
                    //cout << weight << endl;
                    Cargo cargo(n,b,e,w,passby_stations);
                    NAME_CARGO[n] = cargo;
                }
                break;
                default:break;
            }
        }
    }

    void load_file_result()
    {
        //将ROAD和CARGO添加一个空的头，这样其编号和vector的下标对应起来了
        
        istream & f = cin;
        string line;
        bool flag = false;
        while(getline(f,line))
        {
            for(int i = 0;i<line.size();i++)
            {
                if(line[i] == ',')
                    line[i] = ' ';
            }
            if(!flag)
            {
                flag = true;
                stringstream sstream(line);
                
                string tmp;
                sstream>>tmp;
                STATION_NUMBERS = atoi(tmp.c_str());
                sstream>>tmp;
                ROAD_NUMBERS = atoi(tmp.c_str());
                sstream>>tmp;
                CARS_PER_ROAD = atof(tmp.c_str());
                sstream>>tmp;
                LOAD_PER_CAR = atoi(tmp.c_str());
                //测试
                // cout << "STATION_NUMBERS: "<<STATION_NUMBERS<<endl;
                // cout << "ROAD_NUMBERS: " << ROAD_NUMBERS <<endl;
                // cout << "CARS_PER_ROAD: " << CARS_PER_ROAD <<endl;
                // cout << "LOAD_PER_CAR: " << LOAD_PER_CAR <<endl;
                continue;
            }
            //将得到的行进行处理
            
            switch(line[0])
            {
                case 'Z':
                {
                    stringstream sstream(line);
                    string name;
                    sstream>>name;
                    if(name.empty())
                        DEBUG("Z name");
                    name = name.substr(1,name.size()-1);
                    int n = atoi(name.c_str());

                    string workers;
                    sstream>>workers;
                    Station station(n,atoi(workers.c_str()));
                    NAME_STATION.insert({n,station});
                }
                break;
                case 'R':
                {
                    stringstream sstream(line);
                    string name;
                    sstream>>name;
                    if(name.empty())
                        DEBUG("R name");
                    name = name.substr(1,name.size()-1);
                    int n = atoi(name.c_str());

                    string station1;
                    sstream>>station1;
                    if(station1.empty())
                        DEBUG("R station1");
                    station1 = station1.substr(1,station1.size()-1);
                    int s1 = atoi(station1.c_str());

                   
                    string station2;
                    sstream>>station2;
                    if(station2.empty())
                        DEBUG("R station2");
                    station2 = station2.substr(1,station2.size()-1);
                    int s2 = atoi(station2.c_str());
                    Road road(n,s1,s2);
                    //站点构图
                    NAME_STATION[s1].station_bridge[s2]=n;
                    NAME_STATION[s2].station_bridge[s1]=n;
                    NAME_STATION[s1].roads.insert(n);
                    NAME_STATION[s2].roads.insert(n);
                    NAME_STATION[s1].road_car[n] = vector<bool>(CARS_PER_ROAD,false);
                    NAME_STATION[s2].road_car[n] = vector<bool>(CARS_PER_ROAD,false);
                    //收入NAME_ROAD
                    NAME_ROAD[n] = road;
                }
                break;
                case 'G':
                {
                    stringstream sstream(line);
                    string name;
                    sstream >> name;
                    if(name.empty())
                        DEBUG("G name");
                    name = name.substr(1,name.size()-1);
                    int n = atoi(name.c_str());
                    //测试
                    //cout<<"Cargo "<<n<<"has been put"<<endl;
                    string begin;
                    sstream >> begin;
                    if(begin.empty())
                        DEBUG("G begin");
                    begin = begin.substr(1,begin.size()-1);
                    int b = atoi(begin.c_str());

                    string end;
                    sstream >> end;
                    if(end.empty())
                        DEBUG("G end");
                    end = end.substr(1,end.size()-1);
                    int e = atoi(end.c_str());

                    string weight;
                    sstream >> weight;
                   

                    vector<int> passby_stations;
                    string tmp;
                    while(sstream>>tmp)
                    {
                        if(tmp.empty())
                            DEBUG("G tmp");
                        if(tmp == "null")
                            break;
                        tmp = tmp.substr(1,tmp.size()-1);
                        int t = atoi(tmp.c_str());
                        
                        passby_stations.push_back(t);
                    }
                    double w = atof(weight.c_str());
                    //测试
                    //cout << weight << endl;
                    Cargo cargo(n,b,e,w,passby_stations);
                    NAME_CARGO[n] = cargo;
                }
                break;
                default:break;
            }
        }
    }
   
    //测试数据是否正确读入
    void test_input()
    {   
        ofstream out1("STATION.txt",ios::out|ios::trunc);
        if(!out1.is_open())
        {
            cout << "创建文件STATION失败"<<endl;
            exit(1);
        }
        for(auto iter = NAME_STATION.begin();iter!=NAME_STATION.end();iter++)
        {
            out1 <<iter->first <<" "<<iter->second.workers<<endl;
        }
        
        ofstream out2("ROAD.txt",ios::trunc|ios::out);
        if(!out2.is_open())
        {
            cout << "创建文件ROAD失败"<<endl;
            exit(1);
        }
        if(NAME_ROAD.empty())
            DEBUG("NAME_ROAD");
        cout << NAME_ROAD.size()<<endl;
        for(auto iter = NAME_ROAD.begin();iter!=NAME_ROAD.end();iter++)
        {
            out2 <<iter->first <<" "<<iter->second.station1<<" "<<
            iter->second.station2<<endl;
        }
        ofstream out3("Cargo.txt",ios::trunc|ios::out);
        if(!out3.is_open())
        {
            cout << "创建文件Cargo失败"<<endl;
            exit(1);
        }
        for(auto iter = NAME_CARGO.begin();iter!=NAME_CARGO.end();iter++)
        {
            out3 << iter->first<<" "<<iter->second.begin<<" "<<
            iter->second.end<<" "<<iter->second.weight<<endl;
        }
    }

    //测试各路口的道路
    void test_station_roads()
    {
        for(int i = 0;i<NAME_STATION.size();i++)
        {
            cout << "Z" <<i<<":  ";
            set<int> & roads = NAME_STATION[i].roads;
            for(auto iter = roads.begin();iter!=roads.end();iter++)
            {
                cout << *iter <<"  ";
            }
            cout << endl;
        }
    }

    //测试迪杰斯特拉算法
    void test_dijkstra()
    {
        // Cargo cargo = NAME_CARGO[1];
        // cargo.dijkstra();
        // cout << "Cargo"<< 1 <<"("<<cargo.begin<<","<<cargo.end<<")"<<": ";
        // for(int j = 0;j<cargo.path.size();j++)
        // {
        //     cout << cargo.path[j] << "  ";
        // }
        clock_t start = clock();
        for(auto iter = NAME_CARGO.begin();iter!=NAME_CARGO.end();iter++)
        {
            Cargo cargo = NAME_CARGO[iter->first];
            
            cargo.dijkstra();
            
            cout << "Cargo"<< iter->first <<"("<<cargo.begin<<","<<cargo.end<<")"<<": ";
            for(int j = 0;j<cargo.path.size();j++)
            {
                cout << cargo.path[j] << "  ";
            }
            cout << endl;
            
        }
        clock_t end = clock();
        double endtime = (double)(end-start)/CLOCKS_PER_SEC;
        cout << "Total time " <<endtime*1000<<" ms"<<endl;
        
    }

    
    //开始运行，不变道
    void start_run()
    {
        ofstream out1("result.txt",ios::out|ios::trunc);
        if(!out1.is_open())
        {
            cout << "创建文件result失败"<<endl;
            exit(1);
        }
        //失败个数，失败总重
        //cout << setiosflags(ios::fixed) <<setprecision(3) << n << endl;
        cout <<setiosflags(ios::fixed)<<setprecision(3)<<"total weight "<<total_weight<<endl;
        out1<<fail_cargo<<","<<setiosflags(ios::fixed)<<setprecision(3)<<total_weight<<endl;
        for(auto iter = NAME_CARGO.begin();iter!=NAME_CARGO.end();iter++)
        {
            
            Cargo & cargo = NAME_CARGO[iter->first];
            vector<int> & roads = cargo.passby_roads;
            vector<int> & truck_numbers = cargo.truck_numbers;
            out1<<"G"<<cargo.No<<endl;
            if(!cargo.is_success)
            {
                out1<<"null"<<endl<<"null"<<endl;
                continue;
            }
            //roads
            for(int j = 0;j<roads.size();j++)
            {
                if(j!=roads.size()-1)
                    out1<<"R"<<roads[j]<<",";
                else
                    out1<<"R"<<roads[j]<<endl;
            }
            //轨道全部置为col[0],数量与road.size()相同
            for(int j = 0;j<truck_numbers.size();j++)
            {
                if(j!=roads.size()-1)
                    out1<<truck_numbers[j]<<",";
                else
                    out1<<truck_numbers[j]<<endl;
            }
        }     
    }
    void start_run_result()
    {
        ostream & out1 = cout;
        //失败个数，失败总重
        //cout << setiosflags(ios::fixed) <<setprecision(3) << n << endl;
        //cout <<setiosflags(ios::fixed)<<setprecision(3)<<"total weight "<<total_weight<<endl;
        out1<<fail_cargo<<","<<setiosflags(ios::fixed)<<setprecision(3)<<total_weight<<endl;
        for(auto iter = NAME_CARGO.begin();iter!=NAME_CARGO.end();iter++)
        {
            
            Cargo & cargo = NAME_CARGO[iter->first];
            vector<int> & roads = cargo.passby_roads;
            vector<int> & truck_numbers = cargo.truck_numbers;
            out1<<"G"<<cargo.No<<endl;
            if(!cargo.is_success)
            {
                out1<<"null"<<endl<<"null"<<endl;
                continue;
            }
            //roads
            for(int j = 0;j<roads.size();j++)
            {
                if(j!=roads.size()-1)
                    out1<<"R"<<roads[j]<<",";
                else
                    out1<<"R"<<roads[j]<<endl;
            }
            //轨道全部置为col[0],数量与road.size()相同
            for(int j = 0;j<truck_numbers.size();j++)
            {
                if(j!=roads.size()-1)
                    out1<<truck_numbers[j]<<",";
                else
                    out1<<truck_numbers[j]<<endl;
            }
        }     
    }

    //路径的信息，用于ksp算法
    struct Path
    {
        vector<int> path;
        double val;
        friend bool operator<(const Path & r,const Path & n) {return r.val<=n.val;}
        //两者相等必须保证val相同和path相同
        // bool operator==(const Path & r) const{
        //     if(val!=r.val)
        //         return false;
        //     for(int i = 0,j=0;i<path.size()&&j<r.path.size();i++,j++)
        //     {
        //         if(path[i]!=r.path[j])
        //             return false;
        //     }
        //     return true;
        // }
        Path(vector<int> p,double v):path(p),val(v){}
        
    };
    //ksp算法是心啊
    void ksp(int Cargo_id,set<Path> & result,int k)
    {
        
        Cargo & cargo = NAME_CARGO[Cargo_id];
       
        //i = 1表示换path[0]到path[1]之间的道路
        int i = 1;
        //防止重复，并且按照val从小到大排序
        set<Path> sP;
        //cargo的起始dijstra
        sP.insert({cargo.path,cargo.val});
        //测试
        // for(auto iter = sP.begin();iter!=sP.end();iter++)
        // {
        //     int l = iter->path[0];
        //     cout << iter->path[0] <<endl;
        // }
        //k>0,i点不能越界，sP不能为空
        int j = k;
        //测试
        //cout<<"Cargo "<<cargo.No<<"("<<cargo.begin<<","<<cargo.end<<")"<<endl;
        while(j>0&&!sP.empty())
        {
            //获得val最小的当前path
            int size = sP.size();
            //测试
            //cout << "sP cur size: "<<size<<endl;
            //暂存得到的新路径
            set<Path> tmp;
            while(!sP.empty())
            {
                vector<int> path = sP.begin()->path ;
                
                sP.erase(sP.begin());
                if(i>path.size()-1)
                {
                    //测试
                    //cout <<"this path cannot been search"<<endl;
                    continue;
                }
                    
                int cur = path[i-1];
                int after = path[i];
                Station & station = NAME_STATION[cur];
                //遍历每一个cur周围的station
                map<int,int> & station_station = station.station_bridge;
                for(auto iter = station_station.begin();iter!=station_station.end();iter++)
                {
                    int next_station = iter->first;
                    //测试
                    //cout<<"the Station "<<next_station<<" of "<<cur<<endl;
                    //无法到达的点
                    set<int> unavailable_nodes(path.begin(),path.begin()+i);
                    //测试
                    //cout << "unavailable nodes:  ";
                    // for(auto it = unavailable_nodes.begin();it!=unavailable_nodes.end();it++)
                    // {
                    //     cout << *it << "  ";
                    // }
                    //cout << endl;
                    //把after点去掉
                    if(next_station == after)
                        continue;
                    if(unavailable_nodes.count(next_station)>0)
                        continue;
                    //对该结点进行dijkstra计算（这里的dijkstra需要限制不能经0，i-1的点）
                   
                    //不经过某些点的dijstra算法
                    double val = 0.0;
                    //从next_station到end的算法
                    vector<int> nextStation_end;
                    dijkstra(next_station,cargo.end,nextStation_end,val,unavailable_nodes);
                    //测试
                    //cout << "next_station to end:  ";
                    // for(int r = 0;r<nextStation_end.size();r++)
                    // {
                    //     cout << nextStation_end[r] <<"  ";
                    // }
                    // cout << endl;
                    //判断输出路径是否为空,如果为空，则continue
                    if(nextStation_end.empty())
                        continue;
                    //测试
                    //cout<<"all after val:  "<< val<<endl;
                    //将前半段和后半段的路径拼接，并计算val,然后压入
                    //加上前半段路径总权重
                    for(int m = 1;m<i;m++)
                    {
                        Station & station_k = NAME_STATION[path[m-1]]; 
                        Road & road = NAME_ROAD[station_k.station_bridge[path[m]]];
                        val+=road.k;
                    }
                    //测试
                    //cout<<"all pre val + all after val:  "<<val<<endl;
                    //加上连接道路的权重
                    Road & bridge = NAME_ROAD[station_station[next_station]];
                    val+=bridge.k;
                    //测试
                    //cout<<"all val:  "<<val <<endl;
                    vector<int> path_result;
                    path_result.insert(path_result.end(),path.begin(),path.begin()+i);
                    path_result.insert(path_result.end(),nextStation_end.begin(),nextStation_end.end());
                    //测试
                    //cout <<"path_result:  ";
                    // for(int r = 0;r<path_result.size();r++)
                    // {
                    //     cout << path_result[r]<<"  ";
                    // }
                    //cout << endl;
                    tmp.insert({path_result,val});
                    result.insert({path_result,val});
                    j--;
                    if(j==0)
                        return;
                    //cout << endl;
                }
                //cout << endl;
            }
            //更新sP
            sP = tmp;
            //换下一个结点
            i++;
        }
            
        
    }
    //不经过特定道路的dijkstra
    void dijkstra(int begin,int end,vector<int> & result,double & val,set<int> & unavailable_nodes)
    {
        //如果该路径已经存在，就直接调取
        // if(BEGIN_ROUTE.find(begin)!=BEGIN_ROUTE.end())
        // {
        //     //vector<double> & Route = BEGIN_ROUTE[begin].first;
        //     vector<int> & pre = BEGIN_ROUTE[begin].second;
        //     int i = end;
        //     while(i!=-1)
        //     {
        //         path.push_back(i);
        //         i = pre[i];
        //     }
        //     //reverse(passby_roads.begin(),passby_roads.end());
        //     reverse(path.begin(),path.end());
        //     return;
        // }
        int size = NAME_STATION.size();
        vector<bool> vis(size,false);
        
        //用于记录某节点的前结点
        vector<int> pre(size);
        //dis表示cargo的位置到某一节点的位置
        vector<double> dis(size,1e8*1.0); 

        //一个结点的信息,pq，默认最大堆，这里与它反着搞
        struct Node{
            double dis;
            int pre_road_id;
            int pos;
            bool operator<(const Node & r) const{return r.dis < dis;}
        };

        

        priority_queue<Node> pq;
        dis[begin] = 0;
        pq.push({0.0,-1,begin});
        pre[begin] = -1;

        

        while(!pq.empty())
        {
            Node cur_node = pq.top();
            pq.pop();
            if(vis[cur_node.pos])
                continue;
            vis[cur_node.pos] = true;
            
            //遍历临接站点，找最小临接矩阵
            Station station = NAME_STATION[cur_node.pos];
            set<int> & roads = station.roads;
            if(roads.empty())
                DEBUG("station.road is empty!");
            for(auto iter = roads.begin();iter!=roads.end();iter++)
            {
                Road road = NAME_ROAD[*iter];
                //该公路的另一端
                int another_station = (road.station1 == station.name?road.station2:
                    road.station1);
                //如果公路的另一端是不能经过的点，continue
                if(unavailable_nodes.count(another_station)>0)
                {
                    //测试
                    //cout << "you cant't reach station "<<another_station<<endl;
                    continue;
                }
                    
                //如果该点没有被遍历过
                //测试
                
                if(!vis[another_station])
                {
                    //cout << "here" <<endl;
                    int pos = cur_node.pos;
                    int as = another_station;
                    double val = road.k;
                    if(dis[pos]+val<dis[as])
                    {
                        dis[as] = dis[pos]+val;
                        pq.push({dis[as],pos,as});
                        //测试
                        pre[as] = pos;
                    }
                }
            }
        }

        
        
        //输出路径之前校验是否可达
        if(dis[end]>=10000000)
        {
            //cout << "Cargo "<<No <<" can't reach end\n";
            return;
        }
        val = dis[end];
        //BEGIN_ROUTE[begin] = {dis,pre};
        //输出路径
        int i = end;
        while(i!=-1)
        {
            result.push_back(i);
            i = pre[i];
        }
        reverse(result.begin(),result.end());
        
    }

    //开始处理数据
    void start_load()
    {
        //第一阶段，处理普通货物
        //普通货物的源宿合并
        Merge();
        //记录第一次因为中途找不到路而没有成功的货物包
        vector<vector<int>> first_not_success;
        //记录普通货物包因为出发点和终点没有多余的拣货员而不成功的货物包
        vector<vector<int>> normal_no_worker;
        //将计算过的普通货物包的path存起来。
        map<pair<int,int>,vector<vector<int>>> paths;
        for(auto iter = NAME_MERGE_CARGO.begin();iter!=NAME_MERGE_CARGO.end();iter++)
        {
            pair<int,int> begin_end = iter->first;
            Merge_cargo & merge_cargo = iter->second;
            set<pair<double,int>> & cargos = merge_cargo.cargos; 
           
            
            vector<pair<double,vector<int>>> packs;
            while(!cargos.empty())
            {
                double all_weight = 0;
                vector<int> a_pack = merge_cargo.create_pack(all_weight);
                packs.push_back({all_weight,a_pack});
            }
            //贪心，越重的包越优先
            sort(packs.begin(),packs.end(),[&](const pair<double,vector<int>> & a,const pair<double,vector<int>>
            & b){
                return a.first>b.first;
            });
            auto pack_iter = packs.begin();
            while(pack_iter!=packs.end())
            {
                //拿出一车
                vector<int> a_pack = pack_iter->second;
                double weight = pack_iter->first;
                pack_iter++;
                vector<int> path;
                double val = 0.0;
                set<int> no_passing_road;
                //计算路径
                dijkstra(begin_end.first,begin_end.second,path,val,no_passing_road);

                vector<int> origin_path(path.begin(),path.end());
                
                int f = 2;
                //被阻塞的道路
                vector<int> choked_road;
                //进行5次dijkstra选路
                for(int it = 0;it<5;it++)
                {
                    //整条路径的公共货车
                    set<int> public_trucks;
                    for(int pt = 0;pt<CARS_PER_ROAD;pt++)
                    {
                        public_trucks.insert(pt);
                    }
                    if(path.size()<2)
                        continue;
                    //测试
                    //cout << "here"<<endl;
                    int begin = path[0];
                    int end = path[path.size()-1];
                    Station & station_begin = NAME_STATION[begin];
                    Station & station_end = NAME_STATION[end];
                    //判定起始点和终点是否有员工
                    if(station_begin.left_workers==0||station_end.left_workers==0)
                    {
                        //记录
                        normal_no_worker.push_back(a_pack);
                        //这里记录为失败，如果实现了拆包则注释掉
                        fail(a_pack);
                        f = 1;
                        break;
                    }
                    //找到共同轨道
                    //记录遇到堵的道路
                    

                    //筛选整条路径上的公共货车编号
                    for(int j=1; j<path.size();j++)
                    {
                        int cur = path[j-1];
                        int next = path[j];
                        Station & station1 = NAME_STATION[cur];
                        Station & station2 = NAME_STATION[next];
                        int bridge = station1.station_bridge[next];
                        Road & road = NAME_ROAD[bridge];
                        vector<bool> & road_car1 = station1.road_car[bridge];
                        vector<bool> & road_car2 = station2.road_car[bridge];
                        for(int rcw = 0;rcw<CARS_PER_ROAD;rcw++)
                        {
                            if(road_car1[rcw]||road_car2[rcw])
                            {
                                public_trucks.erase(rcw);
                                if(public_trucks.empty())
                                {
                                    //此路已经不通了，将其设为10000000000，选则其它道路
                                    road.k = 100000000;
                                    choked_road.push_back(road.name);
                                    //pt_emp == true;
                                    break;
                                }
                            }
                        }
                    }
                    //如果存在公共货车
                    if(!public_trucks.empty())
                    {
                        //恢复道路的权值
                        for(int m = 0;m<choked_road.size();m++)
                        {
                            Road & road = NAME_ROAD[choked_road[m]];
                            road.k = 1;
                        }
                        //削减人员
                        station_begin.left_workers--;
                        station_end.left_workers--;
                        f = 0;
                        
                        //成功
                        success_mark(a_pack,path,*public_trucks.begin()+1,weight);
                        //cout << "I'm already success"<<endl;
                        break;
                    }
                    else
                    {
                        //否则就重新dijstra，并记录路径
                        paths[{begin,end}].push_back(path);
                        path.clear();
                        dijkstra(begin,end,path,val,no_passing_road);
                    }
                }
                //直通无效，并且有起始，终止工人
                if(f == 2)
                {
                    //恢复路的权值
                    for(int m = 0;m<choked_road.size();m++)
                    {
                        Road & road = NAME_ROAD[choked_road[m]];
                        road.k = 1;
                    }
                    //fail(a_pack);
                    //将第一个cargo的path赋值
                    NAME_CARGO[a_pack[0]].path = origin_path;
                    //fail(a_pack);
                }
            }
            
        }

        //第二阶段，处理因为中途找不到公共道路而没有成功的货物包，这时候中途使用拣货员
        for(int fnp = 0;fnp<first_not_success.size();fnp++)
        {
            //测试
            //cout << "第一阶段结束，进入第二阶段"<<endl;
            vector<int> Fnp = first_not_success[fnp];
            //测试
            if(Fnp.empty())
            {
                DEBUG("Fnp empty");
            }
            int first_cargo = Fnp[0];
            //测试
            //cout << Fnp <<endl;
            Cargo & cargo = NAME_CARGO[first_cargo];
            if(paths.empty())
                DEBUG("paths empty");
            if(run_path(first_cargo,paths[{cargo.begin,cargo.end}])!=0)
            {
                //测试
                //cout << "second no passing"<<endl;
                fail(Fnp);
                continue;
            }
            //测试
            cout << "second success"<<endl;
            success_mark(Fnp);
        }
        //第三阶段，处理特殊货物
        //合并路径相同的特殊货物
        Merge_Route();
        //特殊货物包因为没有公共道路而失败
        vector<vector<int>> special_not_success;
        //特殊货物包因为
        vector<vector<int>> special_no_enough_worker;
        for(auto iter = NAME_MERGE_ROUTE.begin();iter!=NAME_MERGE_ROUTE.end();iter++)
        {
            vector<int> path = iter->first;
            Merge_route & merge_route = iter->second;
            set<pair<double,int>> & cargos = merge_route.cargos; 

            vector<pair<double,vector<int>>> packs;
            while(!cargos.empty())
            {
                double all_weight = 0;
                vector<int> a_pack = merge_route.create_pack(all_weight);
                packs.push_back({all_weight,a_pack});
            }
            sort(packs.begin(),packs.end(),[&](const pair<double,vector<int>> & a,const pair<double,vector<int>>
            & b){
                return a.first>b.first;
            });
            auto pack_iter = packs.begin();
            
            while(pack_iter!=packs.end())
            {
                //测试
                //cout << "in loop"<<endl;
                //选路
                //拿出一车
                vector<int> a_pack = pack_iter->second;
                double weight = pack_iter->first;
                pack_iter++;
                //vector<int> path;
                double val = 0.0;
                set<int> no_passing_road;
                //计算路径
                //dijkstra(begin_end.first,begin_end.second,path,val,no_passing_road);

                //vector<int> origin_path(path.begin(),path.end());
                
                int f = 2;
                //被阻塞的道路
                vector<int> choked_road;
                for(int it = 0;it<1;it++)
                {
                    set<int> public_trucks;
                    for(int pt = 0;pt<CARS_PER_ROAD;pt++)
                    {
                        public_trucks.insert(pt);
                    }
                    if(path.size()<2)
                        continue;
                    //测试
                    //cout << "here"<<endl;
                    int begin = path[0];
                    int end = path[path.size()-1];
                    Station & station_begin = NAME_STATION[begin];
                    Station & station_end = NAME_STATION[end];
                    //判定是否合适
                    if(station_begin.left_workers==0||station_end.left_workers==0)
                    {
                        fail(a_pack);
                        special_no_enough_worker.push_back(a_pack);
                        f = 1;
                        break;
                    }
                    //找到共同轨道
                    //记录遇到堵的道路
                    
                    for(int j=1; j<path.size();j++)
                    {
                        int cur = path[j-1];
                        int next = path[j];
                        Station & station1 = NAME_STATION[cur];
                        Station & station2 = NAME_STATION[next];
                        int bridge = station1.station_bridge[next];
                        Road & road = NAME_ROAD[bridge];
                        vector<bool> & road_car1 = station1.road_car[bridge];
                        vector<bool> & road_car2 = station2.road_car[bridge];
                        for(int rcw = 0;rcw<CARS_PER_ROAD;rcw++)
                        {
                            if(road_car1[rcw]||road_car2[rcw])
                            {
                                public_trucks.erase(rcw);
                                if(public_trucks.empty())
                                {
                                    //此路已经不通了，将其设为10000000000，选则其它道路
                                    road.k = 100000000;
                                    choked_road.push_back(road.name);
                                    //pt_emp == true;
                                    break;
                                }
                            }
                        }
                    }
                    //测试
                    //cout << "out of select car "<<"  public_trucks.size():"<<public_trucks.size()<<endl;
                    if(!public_trucks.empty())
                    {
                        //恢复道路的权值
                        for(int m = 0;m<choked_road.size();m++)
                        {
                            Road & road = NAME_ROAD[choked_road[m]];
                            road.k = 1;
                        }
                        station_begin.left_workers--;
                        station_end.left_workers--;
                        f = 0;
                        //测试
                        //cout << "I'm going to success"<<endl;
                        success_mark(a_pack,path,*public_trucks.begin()+1,weight);
                        //cout << "I'm already success"<<endl;
                        break;
                    }
                    else
                    {
                        //记为失败
                        special_not_success.push_back(a_pack);
                        fail(a_pack);
                    }
                }
            }
            
        }
        //第四阶段，未完成，存在bug
        //第四阶段，处理第一阶段没有拣货员的包，拆包重装
        // for(int i = 0;i<normal_no_worker.size();i++)
        // {
        //     for(int j = 0;j<normal_no_worker[i].size();j++)
        //     {
                
        //         int cargo_id = normal_no_worker[i][j];
        //         Cargo & cargo = NAME_CARGO[cargo_id];
                
        //         if(NAME_STATION[cargo.end].left_workers<=0)
        //         {
        //             fail(cargo_id);
        //             continue;
        //         }
        //          //测试
        //         // if(cargo_id == 160)
        //         // {
        //         //     cout << "cargo 160 size: "<<cargo.path.size()<<endl;
        //         //     sleep(5);
        //         // }

        //         Station & station_begin = NAME_STATION[cargo.begin];
        //         set<int> & roads = station_begin.roads;
                
        //         //{next_station,available_trucks}
        //         //能够使用的站点，卡车
        //         map<int,vector<int>> station_availableTrucks;
        //         for(auto it : roads)
        //         {
        //             Road & road = NAME_ROAD[it];
        //             int station_next_id = 
        //             (road.station1 == station_begin.name?road.station2:road.station1);
        //             Station & station_next = NAME_STATION[station_next_id];
        //             if(station_next.left_workers<2)
        //                 continue;
        //             vector<bool> & begin_road_car_worker = station_begin.road_car_worker[it]; 
        //             vector<bool> & next_road_car_worker = station_next.road_car_worker[it]; 
        //             for(int cw = 0;cw<begin_road_car_worker.size();cw++)
        //             {
        //                 Truck & truck = road.trucks[cw];
        //                 if(!begin_road_car_worker[cw])
        //                     continue;
        //                 if(truck.cur_weight+cargo.weight>100)
        //                     continue;
        //                 if(next_road_car_worker[cw])
        //                 {
        //                     station_availableTrucks[station_next_id].push_back(cw+1);
        //                 }
        //                 else if(station_next.left_workers>=2)
        //                 {
        //                     station_next.left_workers--;
        //                     next_road_car_worker[cw] = true;
        //                     //标记为有车
        //                     station_next.road_car[it][cw] = true;
        //                     station_availableTrucks[station_next_id].push_back(cw+1);
        //                 }
        //             }
                    
        //         }
        //         //每一条路随机取一个
        //         vector<pair<int,int>> foo;
        //         for(auto sa_it:station_availableTrucks)
        //         {
        //             if(sa_it.second.empty())
        //                 continue;
        //             random_shuffle(sa_it.second.begin(),sa_it.second.end());
        //             foo.push_back({sa_it.first,sa_it.second[0]});
        //         }
        //         int fl = 2;
        //         for(auto f : foo)
        //         {
        //             int next_station_id = f.first;
        //             int truck_id = f.second;

        //             //第一步，dijkstra
        //             vector<int> path;
        //             double val;
        //             set<int> cannot_pass;
        //             cannot_pass.insert(cargo.begin);
        //             //以next_station为起点，begin为cannot_pass
        //             dijkstra(next_station_id,cargo.end,path,val,cannot_pass);
        //             if(path.empty())
        //                 continue;
        //             cargo.path = path;

        //             cargo.path.insert(cargo.path.begin(),cargo.begin);

                    
        //             //被阻塞的道路
        //             vector<int> choked_road;
        //             for(int it = 0;it<1;it++)
        //             {
        //                 set<int> public_trucks;
        //                 for(int pt = 0;pt<CARS_PER_ROAD;pt++)
        //                 {
        //                     public_trucks.insert(pt);
        //                 }
        //                 if(path.size()<3)
        //                     continue;
        //                 //测试
        //                 //cout << "here"<<endl;
        //                 int begin = path[0];
        //                 int end = path[path.size()-1];
        //                 Station & station_begin = NAME_STATION[begin];
        //                 Station & station_end = NAME_STATION[end];
                        
        //                 for(int j=1; j<path.size();j++)
        //                 {
        //                     int cur = path[j-1];
        //                     int next = path[j];
        //                     Station & station1 = NAME_STATION[cur];
        //                     Station & station2 = NAME_STATION[next];
        //                     int bridge = station1.station_bridge[next];
        //                     Road & road = NAME_ROAD[bridge];
        //                     vector<bool> & road_car1 = station1.road_car[bridge];
        //                     vector<bool> & road_car2 = station2.road_car[bridge];
        //                     vector<bool> & road_worker_car2 = station2.road_car_worker[bridge];
        //                     for(int rcw = 0;rcw<CARS_PER_ROAD;rcw++)
        //                     {
        //                         if(road_car1[rcw]||road_car2[rcw])
        //                         {
        //                             public_trucks.erase(rcw);
        //                             if(public_trucks.empty())
        //                             {
        //                                 //此路已经不通了，将其设为10000000000，选则其它道路
        //                                 road.k = 100000000;
        //                                 choked_road.push_back(road.name);
        //                                 //pt_emp == true;
        //                                 break;
        //                             }
        //                         }
        //                     }
        //                 }
        //                 //测试
        //                 //cout << "out of select car "<<"  public_trucks.size():"<<public_trucks.size()<<endl;
        //                 if(!public_trucks.empty())
        //                 {
        //                     //恢复道路的权值
        //                     for(int m = 0;m<choked_road.size();m++)
        //                     {
        //                         Road & road = NAME_ROAD[choked_road[m]];
        //                         road.k = 1;
        //                     }

        //                     //判定使用的结尾类型
        //                     // Station & pre_end = NAME_STATION[path[path.size()-2]];
        //                     // int brige_id = station_end.station_bridge[pre_end.name];
        //                     // vector<bool> & road_car = station_end.road_car[brige_id];
        //                     // vector<bool> & road_car_worker = station_end.road_car_worker[brige_id];

        //                     int truck_no = *public_trucks.begin()+1;
        //                     station_end.left_workers--;
        //                     station_begin.left_workers--;
                                
        //                     fl = 0;
        //                     //测试
        //                     //cout << "I'm going to success"<<endl;
        //                     //cargo.id,begin_truck_id,mid_truck_id
        //                     success_mark(cargo.No,truck_id,truck_no);
        //                     //cout << "I'm already success"<<endl;
        //                     break;
        //                 }
        //                 else
        //                 {
        //                     continue;
        //                 }
        //             }
        //             if(fl == 0)
        //                 break;
        //         }
        //         if(fl == 2)
        //             fail(cargo.No);
        //     }
        // }
        
    }

    

    //成功返回0，因起始点无车返回1，因起始点无员工返回2，因中途无法调度返回3，因对面站无车辆接收返回4，
    //因终点站没有足够员工返回5
    //注意，先不标记，标记等成功再标记
    int run_path(int cargo_id,vector<vector<int>> paths)
    {
        //测试
        //cout<< "here"<<endl;
        int i = cargo_id;
        Cargo & cargo = NAME_CARGO[i];
        vector<int> & path = cargo.path;
        //测试
        //cout << "paths size: "<<paths.size()<<endl;
        for(int p = 0;p<paths.size();p++)
        {
            //测试
            // if(paths.empty())
            //     cout << "该路径不存在" <<endl;
            path = paths[p];
            if(path.empty())
                DEBUG("path empty\n");
            //测试
            //cout << "here is p "<<p<<endl;
            //用于判断是否成功
            bool f = true;
            for(int j=0; j<path.size()-1;j++)
            {
                //当前站点ID
                int cur = path[j];
                //下一站点ID
                int next = path[j+1];
                
                Station & station = NAME_STATION[cur];
                Station & station_next = NAME_STATION[next];
                //两站点的桥ID
                int bridge = station.station_bridge[next];
                //两站点的货运情况
                vector<bool> & road_car = station.road_car[bridge];
                vector<bool> & next_road_car = station_next.road_car[bridge];
                //两站点的货运工情况
                
                //两站点之间的桥
                Road & road = NAME_ROAD[bridge];
                
                vector<Truck> & trucks = road.trucks;
                //剩余员工数
                int & left_worker = station.left_workers;
                int & next_left_worker = station_next.left_workers;

                //起始点或者终点没有拣货员，就重算
                Station & begin = NAME_STATION[cargo.begin];
                Station & end = NAME_STATION[cargo.end];
                if(begin.left_workers==0||end.left_workers==0)
                {
                    //测试起始终止点没有拣货员，无效
                    cout << "no enough workers\n";
                    return 1;
                }
                if(j == 0)
                {
                    //测试
                    //cout << "第 "<< p<<" 路径"<<endl;
                    bool flag = false;
                    //测试
                    // if(road_car.size()>CARS_PER_ROAD)
                    // {
                    //     cerr << "Cargo "<<i<<" in road "<<road.name<<" has out of range cars"<<endl;
                    //     exit(1);
                    // }
                    for(int k = 0;k<road_car.size();k++)
                    {
                        //如果车没人用
                        if(!road_car[k])
                        {   
                            //找对面相对应的空位，如果不为空，continue
                            if(next_road_car[k])
                                continue;
                            //测试
                            //cout << "find start "<<k<<endl;
                            //不需要判定下一条路径是不是站尾
                            //肯定有剩余员工
                            if(left_worker>0)
                            {
                                cargo.truck_numbers.push_back(k+1);
                                flag = true;
                                //测试
                                //cout << "first is done!"<<endl;
                                break;
                            }
                            //如果起点没有员工,返回
                            else if(left_worker == 0)
                            {
                                // for(auto It = station.station_bridge.begin();It!=station.station_bridge.end();It++)
                                // {
                                //     Road & r = NAME_ROAD[It->second];
                                //     r.k++;
                                // }
                                //测试
                                //cout << "no begining workers"<<endl;
                                return 1;
                            }
                        }
                    }
                    //没找到，说明没有可达的车，设定为不可到达，放弃该路段
                    if(!flag)
                    {
                        //road.k = 10000000000;
                        cargo.truck_numbers.clear();
                        f = false;
                        //测试
                        cout << "head has "<<"no common car!"<<endl;
                        break;
                    }
                }
                //处于非起始点，站点需要对其进行调度,这里开始使用拣运工
                else
                {
                    //获得货物当前所在车辆的编号
                    int truck_no = cargo.truck_numbers.back();
                    //寻找两个staion的共同空闲车辆
                    vector<bool> common_cars(CARS_PER_ROAD,false);
                    for(int cc = 0;cc<road_car.size();cc++)
                    {
                        if(!road_car[cc]&&!next_road_car[cc])
                            common_cars[cc] = true;
                    }
                    //如果没有共同的空闲车辆,则该路应当被封
                
                    
                    //如果相同的车辆和truck_no-1不一样，就必须使用工人
                    if(!common_cars[truck_no-1])
                    {
                        //如果该站点没有剩余工人，加惩罚权重
                        if(left_worker < 2)
                        {
                            // for(auto It = station.station_bridge.begin();It!=station.station_bridge.end();It++)
                            // {
                            //     Road & r = NAME_ROAD[It->second];
                            //     r.k++;
                            // }
                            //road.k = 10000000000;
                            //测试
                            cout << "no enough workers in mid\n";
                            cargo.truck_numbers.clear();
                            f = false;
                            break;
                        }
                        auto it = find(common_cars.begin(),common_cars.end(),true);
                        if(it == common_cars.end())
                        {
                            //road.k = 10000000000;
                            cout << "no common cars\n";
                            cargo.truck_numbers.clear();
                            //测试
                            //cout << "2 "<<"no common car!"<<endl;
                            f = false;
                            break;
                        }
                        //left_worker-=2;
                        int index = it-common_cars.begin();
                        cargo.truck_numbers.push_back(index+1);
                    }
                    //转移至该货车，并将其标记
                    //road_car[truck_no] = true;
                    //记录该货车
                    else 
                    {
                        cargo.truck_numbers.push_back(truck_no);
                    }
                }
            }
            if(f)
                return 0;
        }
        return 1;
        
    }

    //有必经站点的dijkstra算法
    vector<int> multi_dijkstra(int cargoid)
    {
        Cargo & cargo = NAME_CARGO[cargoid];
        vector<int> passby_station = cargo.passby_stations;

        //随机打乱顺序
        //random_shuffle(passby_station.begin(),passby_station.end());
        int begin = cargo.begin;
        int end = cargo.end;
        passby_station.insert(passby_station.begin(),begin);
        passby_station.insert(passby_station.end(),end);
        vector<int> result;
        set<int> unavailable_nodes;
        //测试
        //cout << passby_station.size()<<endl;
        // for(int j = 0;j<passby_station.size();j++)
        // {
        //     cout << passby_station[j]<<endl;
        // }
        for(int i = 0;i<passby_station.size()-1;i++)
        {
            //测试
            //cout << "into loop\n";
            vector<int> path;
            double val;
            dijkstra(passby_station[i],passby_station[i+1],path,val,unavailable_nodes);
            //测试
            //cout << "after dijstra"<<endl;
            //cout << "path.size "<<path.size()<<endl;
            if(path.empty())
            {
                result.clear();
                return result;
            }
            unavailable_nodes.insert(path.begin(),--path.end());
            //测试
            //cout << "unavailabe_nodes.size  "<<unavailable_nodes.size()<<endl;
            //删除最后一个元素
            path.erase(--path.end());
            result.insert(result.end(),path.begin(),path.end());
            //测试
        }
        result.insert(result.end(),cargo.end);
        return result;
    }

    //如果成功，一系列的重载成功函数
    void success_mark(int cargo_id)
    {
        //测试
        //cout << "Cargo "<<cargo_id<<" success"<<endl;
        Cargo & cargo = NAME_CARGO[cargo_id];
        vector<int> & path = cargo.path;
        vector<int> & truck_numbers = cargo.truck_numbers;
        for(int i = 1;i<path.size();i++)
        {
            Station & station1 = NAME_STATION[path[i-1]];
            Station & station2 = NAME_STATION[path[i]];
            int road_id = station1.station_bridge[station2.name];
            cargo.passby_roads.push_back(road_id);
            vector<bool> & v1 = station1.road_car[road_id];
            vector<bool> & v2 = station2.road_car[road_id];

            vector<bool> & v3 = station1.road_car_worker[road_id];
            vector<bool> & v4 = station2.road_car_worker[road_id];

            int truck_no = cargo.truck_numbers[i-1];
            v1[truck_no-1] = true;
            v2[truck_no-1] = true;
            // v3[truck_no-1] = true;
            // v4[truck_no-1] = true;
        }
    }

    void success_mark(int cargo_id,int start_truck,int mid_end_truck)
    {
        //测试
        //cout << "Cargo "<<cargo_id<<" success"<<endl;
        //第一辆车

       
        Cargo & cargo = NAME_CARGO[cargo_id];
        vector<int> & path = cargo.path;
        vector<int> & truck_numbers = cargo.truck_numbers;
        truck_numbers.push_back(start_truck);

        Station & station1 = NAME_STATION[path[0]];
        Station & station2 = NAME_STATION[path[1]];

        int bridge = station1.station_bridge[path[1]];

        cargo.passby_roads.push_back(bridge);

        station1.road_car[bridge][start_truck-1] = true;
        station2.road_car[bridge][start_truck-1] = true;

        station1.road_car_worker[bridge][start_truck-1] = true;
        station2.road_car_worker[bridge][start_truck-1] = true;

        //装载
        Truck & first_truck = NAME_ROAD[bridge].trucks[start_truck-1];
        first_truck.cur_weight+=cargo.weight;

        for(int i = 2;i<path.size();i++)
        {
            Station & station1 = NAME_STATION[path[i-1]];
            Station & station2 = NAME_STATION[path[i]];
            int road_id = station1.station_bridge[station2.name];
            cargo.passby_roads.push_back(road_id);
            vector<bool> & v1 = station1.road_car[road_id];
            vector<bool> & v2 = station2.road_car[road_id];
            int truck_no = mid_end_truck;
            cargo.truck_numbers.push_back(mid_end_truck);
            v1[truck_no-1] = true;
            v2[truck_no-1] = true;
    
            //装车
            Truck & truck = NAME_ROAD[road_id].trucks[truck_no-1];
            truck.cur_weight+=cargo.weight;
            if(truck.cur_weight>100)
            {
                cout << "cargo "<<cargo_id<<"can't be put in "<<endl;
                DEBUG("truck weight");
            }
        }
    }


    //对于一车物品装在truck_id上判断成功
     void success_mark(vector<int> v,vector<int> p,int truck_id,double all_weight)
    {
        //测试
        //cout << "Cargo "<<cargo_id<<" success"<<endl;
        if(v.empty())
            DEBUG("bug:v empty()");
        Cargo & cargo = NAME_CARGO[v[0]];

        cargo.path = p;
        //测试
        //cout<<"cargo id is not out of range"<<endl;
        vector<int> & path = cargo.path;
        if(path.empty())
            DEBUG("path is empty")
        vector<int> & truck_numbers = cargo.truck_numbers;
        truck_numbers = vector<int>(path.size()-1,truck_id);
        //测试
        //cout << "the first cargo's truck_numbers is success"<<endl;
        for(int i = 1;i<path.size();i++)
        {
            Station & station1 = NAME_STATION[path[i-1]];
            Station & station2 = NAME_STATION[path[i]];
            int road_id = station1.station_bridge[station2.name];
            cargo.passby_roads.push_back(road_id);

            //成功说明路更堵了，加权（权值为多少，这个是个学问）
            Road & road = NAME_ROAD[road_id];
            if(road.left_car>0)
            {
                road.k = (CARS_PER_ROAD/(road.left_car*1.0));
                road.left_car--;
            }
            else
            {
                road.k = 10000000000;
            }
            
            if(i == 1)
            {
                station1.road_car_worker[road_id][truck_id-1] = true;
            }
            if(i == path.size()-1)
            {
                station2.road_car_worker[road_id][truck_id-1] = true;
            }
            vector<bool> & v1 = station1.road_car[road_id];
            vector<bool> & v2 = station2.road_car[road_id];
            // vector<bool> & v3 = station1.road_car_worker[road_id];
            // vector<bool> & v4 = station2.road_car_worker[road_id];

            int truck_no = truck_id;
            v1[truck_no-1] = true;
            v2[truck_no-1] = true;
            // v3[truck_no-1] = true;
            // v4[truck_no-1] = true;

            //装车
            Truck & truck = road.trucks[truck_no-1];
            truck.cur_weight+=(all_weight);
        }
        //同步
        for(int j = 1;j<v.size();j++)
        {
            Cargo & cargo_cur = NAME_CARGO[v[j]];
            //测试
            cargo_cur.truck_numbers = cargo.truck_numbers;
            cargo_cur.passby_roads = cargo.passby_roads;
        }
    }

    void success_mark(vector<int> v)
    {
        //测试
        //cout << "Cargo "<<cargo_id<<" success"<<endl;
        if(v.empty())
            DEBUG("bug:v empty()");
        Cargo & cargo = NAME_CARGO[v[0]];
       
        //测试
        //cout<<"cargo id is not out of range"<<endl;
        vector<int> & path = cargo.path;
        if(path.empty())
            DEBUG("path is empty")
        vector<int> & truck_numbers = cargo.truck_numbers;
        //truck_numbers = vector<int>(path.size()-1,truck_id);
        //测试
        //cout << "the first cargo's truck_numbers is success"<<endl;
        for(int i = 1;i<path.size();i++)
        {
            Station & station1 = NAME_STATION[path[i-1]];
            Station & station2 = NAME_STATION[path[i]];
            int road_id = station1.station_bridge[station2.name];
            cargo.passby_roads.push_back(road_id);
            vector<bool> & v1 = station1.road_car[road_id];
            vector<bool> & v2 = station2.road_car[road_id];
            int truck_no = cargo.truck_numbers[i-1];
            int next_truck_no = -1;
            if(i!=path.size()-1)
                next_truck_no = cargo.truck_numbers[i];
            v1[truck_no-1] = true;
            v2[truck_no-1] = true;
            if(i==1)
            {
                station1.left_workers--;
                station2.left_workers--;
            }
            else if(i!=path.size()-1)
            {
                if(truck_no!=next_truck_no)
                    station1.workers-=2;
            }
        }
        //同步
        for(int j = 1;j<v.size();j++)
        {
            Cargo & cargo_cur = NAME_CARGO[v[j]];
            //测试
            // if(cargo_cur.No == 160)
            // {
            //     cout << "success in pakage cargo 160 size: "<<cargo.path.size()<<endl;
            //     sleep(5);
            // }
            cargo_cur.truck_numbers = cargo.truck_numbers;
            cargo_cur.passby_roads = cargo.passby_roads;
        }
    }

    //一车货永久失败
    void fail(vector<int> v)
    {
        for(int i = 0;i<v.size();i++)
        {
            fail(v[i]);
        }
    }
    //如果路径失败，则清理cargo的truck_numbers
    void clear(int cargo_id)
    {
        Cargo & cargo = NAME_CARGO[cargo_id];
        cargo.truck_numbers.clear();
    }

    //如果完全失败，则记录
    void fail(int cargo_No)
    {

        //clear(cargo_No);
        Cargo & cargo = NAME_CARGO[cargo_No];

       
        total_weight+=cargo.weight;
        fail_cargo++;
        cargo.is_success = false;
    }
    
    //普通货物的源宿合并操作
    void Merge()
    {
        for(auto iter = NAME_CARGO.begin();iter!=NAME_CARGO.end();iter++)
        {
            Cargo & cargo = iter->second;
            int begin = cargo.begin;
            int end = cargo.end;
            int weight = cargo.weight;
            int cargo_id = cargo.No;
            if(!cargo.passby_stations.empty())
            {
                need_pass_by_staion.push_back(cargo.No);
                //fail(cargo_id);
                continue;
            }
            if(NAME_MERGE_CARGO.find({begin,end}) == NAME_MERGE_CARGO.end())
                NAME_MERGE_CARGO.insert({{begin,end},Merge_cargo(begin,end)});
            Merge_cargo & merge_cargo = NAME_MERGE_CARGO[{begin,end}];
            merge_cargo.push_back(cargo_id);
        }
    }

    //特殊货物的元素合并操作
    void Merge_Route()
    {
        for(auto iter = need_pass_by_staion.begin();iter!=need_pass_by_staion.end();iter++)
        {
            Cargo & cargo = NAME_CARGO[*iter];
            int cargo_id = cargo.No;
            vector<int> path = multi_dijkstra(cargo_id);
            if(path.empty())
            {
                fail(cargo_id);
                continue;
            }
            cargo.path = path;
            int begin = cargo.begin;
            int end = cargo.end;
            int weight = cargo.weight;
            
            if(NAME_MERGE_ROUTE.find({begin,end}) == NAME_MERGE_ROUTE.end())
                NAME_MERGE_ROUTE.insert({path,Merge_route(path)});
            Merge_route & merge_route = NAME_MERGE_ROUTE[path];
            merge_route.push_back(cargo_id);
        }
    }

    //用于测试
    void left_worker()
    {
        for(auto iter = NAME_STATION.begin();iter!=NAME_STATION.end();iter++)
        {
            cout << "Station "<<iter->first<<" has "<<iter->second.left_workers<<" worker"<<endl;
        }
    }
    //用于测试
    void Cargo_weight()
    {
        for(auto iter = NAME_CARGO.begin();iter!=NAME_CARGO.end();iter++)
        {
            cout <<" weights "<<iter->second.weight<<endl;
        }
    }
    
    
    //用于测试ksp算法
    void test_ksp(int k)
    {
        for(auto iter = NAME_CARGO.begin();iter!=NAME_CARGO.end();iter++)
        {
            int cargo_id = iter->first;
            Cargo & cargo = NAME_CARGO[cargo_id];
            cargo.dijkstra();
            set<Path> sP;
            ksp(cargo_id,sP,k);
            cout <<"Cargo "<<cargo_id<<"("<<cargo.begin<<","<<cargo.end<<"): \n";
            for(auto set_iter = sP.begin();set_iter!=sP.end();set_iter++)
            {
                vector<int> route = set_iter->path;
                double val = set_iter->val;
                cout <<val<<" :   ";
                for(int i = 0;i<route.size();i++)
                {
                    cout << route[i] <<"  ";
                }
                cout << endl;
            }
            cout << endl;
        }
        
    }

 
};


int main()
{
    //Solution s("./test.txt");

    //Solution s1("./测试用例集/case3/topoAndRequest3.txt");
    //Solution s2("./测试用例集/case2/topoAndRequest2.txt"); 
    //Solution s3("./测试用例集/case3/topoAndRequest3.txt");
    //Solution s4("./测试用例集/case4/topoAndRequest4.txt");
    // Solution s5("./测试用例集/case5/topoAndRequest5.txt");
    // Solution s6("./测试用例集/case6/topoAndRequest6.txt");
    // Solution s7("./测试用例集/case7/topoAndRequest7.txt");
     Solution s8("./测试用例集/case8/topoAndRequest8.txt");
    
    return 0;
}



