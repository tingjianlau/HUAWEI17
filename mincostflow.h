#ifndef MINCOSTFLOW_H
#define MINCOSTFLOW_H

#include <iostream>
#include <vector>
#include <queue>
#include <stack>
#include <algorithm>
#include <cstdio>
#include <cstring>

#include "graph.h"

using namespace std;

typedef pair<int, int>P;//first保存最短距离，second保存顶点的编号

struct Edge_MCF
{
    int to, cap, cost, rev;//终点，容量（指残量网络中的），费用，反向边编号
    Edge_MCF(int t, int c, int cc, int r) :to(t), cap(c), cost(cc), rev(r){}
};
class MCF{//最小费用流（指定需求流量）
public:
    int nodeNum;//顶点数
    vector<Edge_MCF> G[maxn];//图的邻接表
    int h[maxn];//顶点的势
    int dist[maxn];//最短距离
    int prevv[maxn];//最短路中的父结点
    int preve[maxn];//最短路中的父边
    Graph graph;//关联的图
    void init(int n){
        nodeNum=n;
        for(int i=0;i<nodeNum;i++){
            G[i].clear();
        }
    }

    void addEdge(int from, int to, int cap, int cost)
    {
        G[from].push_back(Edge_MCF( to, cap, cost, G[to].size()));
        G[to].push_back(Edge_MCF( from, cap, cost, G[from].size() - 1 ));
    }

    void addEdge(vector<Edge_MCF> G1[],int from,int to,int cap,int cost){
        G1[from].push_back(Edge_MCF( to, cap, cost, G[to].size()));
        G1[to].push_back(Edge_MCF( from, cap, cost, G[from].size() - 1 ));
    }

    //获取MCF(转存Graph到MCF的G)
    void createMCF(const Graph &graph){
        this->graph=graph;
        init(graph.nodeNum);
        for (int i = 0; i<graph.nodeNum; i++)
        {
            for(int j=0;j<graph.G[i].size();j++){
                addEdge(graph.G[i][j].from,graph.G[i][j].to,
                            graph.G[i][j].cap,graph.G[i][j].cost);

            }
        }
    }

    //单源单汇最小费用流算法
    int minCostFlow(int s, int t, int f,//起点、终点、流量需求
                    vector<int> minCostPath[],int &m,//输出路径、路径数目
                    int &flowAll,int &minCost,//单源单汇无解时，走最大流量的实际流量输出和费用输出
                    int isUpdate)//是否更新原图
    {
        //为迭代调用，使用图的副本操作
        vector<Edge_MCF> G1[maxn];//图的邻接表
        copy(this->G,this->G+maxn,G1);
        stack<int> stk;
        int pathCnt=0;
        int flowNeed=f;
        int res = 0;
        flowAll=0;
        minCost=0;
        for(int i=0;i<m;i++){
            minCostPath[i].clear();
        }
        fill(h, h + nodeNum, 0);
        while (f>0)//f>0时还需要继续增广
        {
            priority_queue<P, vector<P>, greater<P> >q;
            fill(dist, dist + nodeNum, INF);//距离初始化为INF
            dist[s] = 0;
            q.push(P(0, s));
            while (!q.empty())
            {
                P p = q.top(); q.pop();
                int v = p.second;
                if (dist[v]<p.first)continue;//p.first是v入队列时候的值，dist[v]是目前的值，如果目前的更优，扔掉旧值
                for (int i = 0; i<G1[v].size(); i++)
                {
                    Edge_MCF &e = G1[v][i];
                    if (e.cap>0 && dist[e.to]>dist[v] + e.cost + h[v] - h[e.to])//松弛操作
                    {
                        dist[e.to] = dist[v] + e.cost + h[v] - h[e.to];
                        prevv[e.to] = v;//更新父结点
                        preve[e.to] = i;//更新父边编号
                        q.push(P(dist[e.to], e.to));
                    }
                }
            }

            if (dist[t] == INF){//如果dist[t]还是初始时候的INF，那么说明s-t不连通，不能再增广了
                minCost=res;

                return INF;
            }
            for (int j = 0; j<nodeNum; j++)//更新h
                h[j] += dist[j];
            int d = f;
            for (int x = t; x != s; x = prevv[x]){
                stk.push(x);
                d = min(d, G1[prevv[x]][preve[x]].cap);//从t出发沿着最短路返回s找可改进量
            }
            minCostPath[pathCnt].push_back(s);//加入起点
            //cout<<s<<' ';
            int node;
            while(!stk.empty()){
                node=stk.top();
                stk.pop();
                minCostPath[pathCnt].push_back(node);
                //cout<<node<<' ';
            }
            minCostPath[pathCnt].push_back(this->graph.netToConsumer[node]);//加入消费结点
            if(d<=flowNeed){
                minCostPath[pathCnt].push_back(d);//实际流量
                flowAll+=d;
                //cout<<d<<endl;
            }
            else{
                minCostPath[pathCnt].push_back(flowNeed);
                flowAll+=d;
                //cout<<d<<endl;
            }
            pathCnt++;
            f -= d;
            res += d*h[t];//h[t]表示最短距离的同时，也代表了这条最短路上的费用之和，乘以流量d即可得到本次增广所需的费用
            for (int x = t; x != s; x = prevv[x])
            {
                Edge_MCF &e = G1[prevv[x]][preve[x]];
                e.cap -= d;//修改残量值
                G1[x][e.rev].cap += d;
                //更新原图
                if(isUpdate)G[prevv[x]][preve[x]].cap-=d;
            }

        }
        m=pathCnt;
        return res;
    }

    //多源多汇最小费用流算法
    int multiMinCostFlow(const vector<int> &servers,const vector<int> &consumerNetNodes,
                          int f,vector<int> minCostPath[],int &m,
                          int &flowAll,int &minCost){
         vector<Edge_MCF> G1[maxn];
         copy(this->G,this->G+maxn,G1);
         int superServer=nodeNum;//超级源
         for(int i=0;i<servers.size();i++){
             //超级源与每个服务器建立边：费用0，容量无穷
             addEdge(G1,superServer,servers[i],INF,0);
         }
         int superConsumerNetNode=nodeNum+1;//超级汇
         for(int i=0;i<consumerNetNodes.size();i++){
             //超级汇与每个消费结点所连的网络结点建立边：费用0，容量无穷
             addEdge(G1,consumerNetNodes[i],superConsumerNetNode,graph.consumers[i].flowNeed,0);
         }
         //单次调用超级源到超级汇的最小费用流
         stack<int> stk;
         int pathCnt=0;
         int flowNeed=f;
         int res = 0;
         flowAll=0;
         minCost=0;
         for(int i=0;i<m;i++){
             minCostPath[i].clear();
         }
         fill(h, h + nodeNum+2, 0);
         while (f>0)//f>0时还需要继续增广
         {
             priority_queue<P, vector<P>, greater<P> >q;
             fill(dist, dist + nodeNum+2, INF);//距离初始化为INF
             dist[superServer] = 0;
             q.push(P(0, superServer));
             while (!q.empty())
             {
                 P p = q.top(); q.pop();
                 int v = p.second;
                 if (dist[v]<p.first)continue;//p.first是v入队列时候的值，dist[v]是目前的值，如果目前的更优，扔掉旧值
                 for (int i = 0; i<G1[v].size(); i++)
                 {
                     Edge_MCF &e = G1[v][i];
                     if (e.cap>0 && dist[e.to]>dist[v] + e.cost + h[v] - h[e.to])//松弛操作
                     {
                         dist[e.to] = dist[v] + e.cost + h[v] - h[e.to];
                         prevv[e.to] = v;//更新父结点
                         preve[e.to] = i;//更新父边编号
                         q.push(P(dist[e.to], e.to));
                     }
                 }
             }

             if (dist[superConsumerNetNode] == INF){//如果dist[t]还是初始时候的INF，那么说明s-t不连通，不能再增广了
                 minCost=res;

                 return INF;
             }
             for (int j = 0; j<nodeNum+2; j++)//更新h
                 h[j] += dist[j];
             int d = f;
             for (int x = superConsumerNetNode; x != superServer; x = prevv[x]){
                 stk.push(x);
                 d = min(d, G1[prevv[x]][preve[x]].cap);//从t出发沿着最短路返回s找可改进量
             }
             int node;
             while(!stk.empty()){
                 node=stk.top();
                 stk.pop();
                 minCostPath[pathCnt].push_back(node);
                 //cout<<node<<' ';
             }
             auto end=minCostPath[pathCnt].end();
             minCostPath[pathCnt].erase(end-1);//去掉超级汇点
             node=minCostPath[pathCnt][minCostPath[pathCnt].size()-1];
             minCostPath[pathCnt].push_back(this->graph.netToConsumer[node]);//加入消费结点
             if(d<=flowNeed){
                 minCostPath[pathCnt].push_back(d);//实际流量
                 flowAll+=d;
                 //cout<<d<<endl;
             }
             else{
                 minCostPath[pathCnt].push_back(flowNeed);
                 flowAll+=d;
                 //cout<<d<<endl;
             }
             pathCnt++;
             f -= d;
             res += d*h[superConsumerNetNode];//h[t]表示最短距离的同时，也代表了这条最短路上的费用之和，乘以流量d即可得到本次增广所需的费用
             for (int x = superConsumerNetNode; x != superServer; x = prevv[x])
             {
                 Edge_MCF &e = G1[prevv[x]][preve[x]];
                 e.cap -= d;//修改残量值
                 //G1[x][e.rev].cap += d;

             }

         }
         m=pathCnt;
         return res;
    }
};

#endif // MINCOSTFLOW_H
