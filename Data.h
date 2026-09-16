#ifndef DATA_H
#define DATA_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <map>

using namespace std;

struct Segment {
    int vertex;
    double cost;
};

struct Line {
    int start;              
    int end;                
    double cost;            
    int segments;             
    vector<Segment> segmentList; 
};

struct Coord {
    double x;
    double y;
};

class Data {
public:
    Data();
    Data(string fileName);
    //~Data();

    string instance_name;
    int org_vert;
    int tot_vert;

    int num_depots;
    vector<int> depots;

    int num_lines;
    vector<Line> lines;

    map<int, Coord> coordinates;

    void print() const;

    void exportJSON(const string& fileName) const;

private:
    static string trim(const string& s);
    static string valueAfterColon(const string& s);
    static string jsonEscape(const string& s);
    static double euclidean(const Coord& a, const Coord& b);
};

#endif