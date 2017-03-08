#ifndef GRAPH_H
#define GRAPH_H

#include <iostream>
#include <vector>
#include <queue>
#include <algorithm>
#include <map>
using namespace  std;

#define maxn 1000
#define INF 100000000
#define maxpath 1000
#define maxserver 100

struct Edge
{
    int from, to, cap, flow, cost;//入点、出点、容量、实际流量、流量单价
    Edge()=default;
    Edge(int u, int v, int c, int f,int w):
        from(u), to(v), cap(c), flow(f),cost(w){}
};
struct Consumer{
    int no;//消费节点编号
    int netNode;//所连网络结点编号
    int flowNeed;//流量需求
};
class Graph{
public:
    int nodeNum,edgeNum,consumerNum;//结点数
    vector<Edge> G[maxn];//网络结点邻接表
    vector<Consumer> consumers;//消费节点数组
    map<int,int> netToConsumer;
    //服务器单价
    int serverCost;
    //求最短路
    Edge p[maxn]; // 当前节点单源最短路中的上一条边
    int d[maxn]; // 单源最短路径长

    void init(int n,int m,int k){
        nodeNum=n;
        edgeNum=m;
        consumerNum=k;
        for(int i=0;i<nodeNum;i++){
            G[i].clear();
        }
        consumers.clear();

    }

    //获取图
    void createGraph(char * topo[MAX_EDGE_NUM]){
        string line=topo[0];
        stringstream ss(line);
        int nodeNum;//结点数
        int edgeNum;//边数
        int consumerNum;//消费结点数
        ss>>nodeNum>>edgeNum>>consumerNum;
        init(nodeNum,edgeNum,consumerNum);
        serverCost=strtol(topo[2],NULL,10);
        for(int i=4;i<edgeNum+4;i++){
            Edge e;
            line=topo[i];
            stringstream ss(line);
            ss>>e.from>>e.to>>e.cap>>e.cost;
            G[e.from].push_back(e);
        }
        for(int i=5+edgeNum;i<consumerNum+5+edgeNum;i++){
            Consumer consumer;
            line=topo[i];
            stringstream ss(line);
            ss>>consumer.no>>consumer.netNode>>consumer.flowNeed;
            netToConsumer[consumer.netNode]=consumer.no;//建立网结点和消费结点映射
            consumers.push_back(consumer);
        }
    }

    typedef pair<int, int> HeapNode;
    void dijkstra(int s)
    {
        priority_queue< HeapNode, vector<HeapNode>, greater<HeapNode> > Q;
        for (int i=0; i<nodeNum; ++i)
            d[i] = INF ;
        d[s] = 0;
        Q.push(make_pair(0, s));
        while (!Q.empty()) {
            pair<int, int> N = Q.top();
            Q.pop();
            int u = N.second;
            if (N.first != d[u]) continue;
            for (int i=0; i<G[u].size(); ++i) {
                Edge &e = G[u][i];
                if (d[e.to] > d[u] + e.cost) {
                    d[e.to] = d[u] + e.cost;
                    p[e.to] = G[u][i];
                    Q.push(make_pair(d[e.to], e.to));
                }
            }
        }
    }
};

#endif // GRAPH_H
