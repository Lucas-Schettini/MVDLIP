#include "Data.h"
#include <set>
#include <cmath>
#include <iomanip>

Data::Data(){
    org_vert = 0;
    tot_vert = 0;
    num_depots = 0;
    num_lines = 0;
}

string Data::trim(const string& s){
    size_t start = s.find_first_not_of(" \t\r\n");
    if(start == string::npos) return "";
    size_t end = s.find_last_not_of(" \t\r\n");
    return s.substr(start, end - start + 1);
}

string Data::valueAfterColon(const string& s){
    size_t pos = s.find(':');
    if(pos == string::npos) return trim(s);
    return trim(s.substr(pos + 1));
}

Data::Data(string fileName) : Data() {
    ifstream readFile(fileName);

    if(!readFile.is_open()){
        cerr << "Arquivo não pode ser aberto" << fileName << endl;
        return;
    }

    string line;

    while(getline(readFile, line)){
        string cleanLine = trim(line);
        if(cleanLine.empty()) continue;

        if(cleanLine.rfind("NOMBRE", 0) == 0){
            instance_name = valueAfterColon(cleanLine);
        }
        else if(cleanLine.rfind("VERTICES ORIGINALES", 0) == 0){
            org_vert = stoi(valueAfterColon(cleanLine));
        }
        else if(cleanLine.rfind("VERTICES TOTALES", 0) == 0){
            tot_vert = stoi(valueAfterColon(cleanLine));
        }
        else if(cleanLine.rfind("DEPOTS", 0) == 0){
            num_depots = stoi(valueAfterColon(cleanLine));
            depots.reserve(num_depots);
            for(int i = 0; i < num_depots; ++i){
                if(!getline(readFile, line)) break;
                depots.push_back(stoi(trim(line)));
            }
        }
        else if(cleanLine.rfind("LINEAS ORIGINALES", 0) == 0){
            num_lines = stoi(valueAfterColon(cleanLine));
            lines.reserve(num_lines);
        }
        else if(cleanLine.rfind("LINE", 0) == 0){
            istringstream iss(cleanLine);
            string tag;
            Line l;
            iss >> tag >> l.start >> l.end >> l.cost >> l.segments;

            l.segmentList.reserve(l.segments);
            for(int i = 0; i < l.segments; ++i){
                if(!getline(readFile, line)) break;
                istringstream segStream(trim(line));
                Segment seg;
                segStream >> seg.vertex >> seg.cost;
                l.segmentList.push_back(seg);
            }

            lines.push_back(l);
        }
        else if(cleanLine.rfind("COORDENADAS", 0) == 0){
            while(getline(readFile, line)){
                string coordLine = trim(line);
                if(coordLine.empty()) continue;

                istringstream coordStream(coordLine);
                int id;
                Coord c;
                coordStream >> id >> c.x >> c.y;
                coordinates[id] = c;
            }
        }
    }

    readFile.close();
}

void Data::print() const {
    cout << "Instancia: " << instance_name << endl;
    cout << "Vertices originais: " << org_vert << endl;
    cout << "Vertices totais: " << tot_vert << endl;

    cout << "Depositos (" << num_depots << "): ";
    for(int d : depots) cout << d << " ";
    cout << endl;

    cout << "Linhas (" << num_lines << "):" << endl;
    for(const auto& l : lines){
        cout << "  LINE " << l.start << " -> " << l.end
             << " | custo=" << l.cost
             << " | segmentos=" << l.segments << endl;
        for(const auto& s : l.segmentList){
            cout << "    -> vertice " << s.vertex << " custo=" << s.cost << endl;
        }
    }

    cout << "Coordenadas (" << coordinates.size() << " vertices):" << endl;
    for(const auto& kv : coordinates){
        cout << "  " << kv.first << ": (" << kv.second.x << ", " << kv.second.y << ")" << endl;
    }
}

string Data::jsonEscape(const string& s){
    string out;
    out.reserve(s.size() + 4);
    for(char c : s){
        switch(c){
            case '"':  out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n";  break;
            case '\r': out += "\\r";  break;
            case '\t': out += "\\t";  break;
            default:   out += c;
        }
    }
    return out;
}

double Data::euclidean(const Coord& a, const Coord& b){
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return sqrt(dx * dx + dy * dy);
}

void Data::exportJSON(const string& fileName) const {
    ofstream out(fileName);
    if(!out.is_open()){
        cerr << "Nao foi possivel criar o arquivo " << fileName << endl;
        return;
    }
    out << setprecision(10);

    struct ReqEdge { 
        int lineId; 
        int u; 
        int v; 
        double de; 
    };
    vector<ReqEdge> requiredEdges;
    set<int> vrSet; 

    for(size_t li = 0; li < lines.size(); ++li){
        const Line& l = lines[li];
        int prev = l.start;
        vrSet.insert(l.start);
        vrSet.insert(l.end);
        for(const Segment& seg : l.segmentList){
            requiredEdges.push_back({(int)li, prev, seg.vertex, seg.cost});
            vrSet.insert(prev);
            vrSet.insert(seg.vertex);
            prev = seg.vertex;
        }
        if(l.segmentList.empty()){
            requiredEdges.push_back({(int)li, l.start, l.end, l.cost});
        }
    }

    set<int> dSet(depots.begin(), depots.end());
    dSet.insert(0);

    struct Arc { 
        int i; 
        int j; 
        double dij; 
    };
    vector<Arc> arcs;
    for(int i : dSet){
        for(int j : dSet){
            if(i == j) continue;
            auto itI = coordinates.find(i);
            auto itJ = coordinates.find(j);
            if(itI == coordinates.end() || itJ == coordinates.end()) continue;
            arcs.push_back({i, j, euclidean(itI->second, itJ->second)});
        }
    }

    out << "{\n";

    out << "  \"instance\": {\n";
    out << "    \"name\": \"" << jsonEscape(instance_name) << "\",\n";
    out << "    \"vertices_originais\": " << org_vert << ",\n";
    out << "    \"vertices_totais\": " << tot_vert << "\n";
    out << "  },\n";

    out << "  \"sets\": {\n";

    out << "    \"V\": [";
    {
        bool first = true;
        for(const auto& kv : coordinates){
            if(!first) out << ", ";
            out << kv.first;
            first = false;
        }
    }
    out << "],\n";

    out << "    \"D\": [";
    for(size_t i = 0; i < depots.size(); ++i){
        out << (i ? ", " : "") << depots[i];
    }
    out << "],\n";

    out << "    \"VR\": [";
    {
        bool first = true;
        for(int v : vrSet){
            if(!first) out << ", ";
            out << v;
            first = false;
        }
    }
    out << "],\n";

    out << "    \"T\": null,\n";
    out << "    \"_T_preencher\": \"numero de caminhoes disponiveis (|T|)\",\n";
    out << "    \"K\": null,\n";
    out << "    \"_K_preencher\": \"numero maximo de voos por parada (|K|)\"\n";
    out << "  },\n";

    out << "  \"required_edges\": [\n";
    for(size_t i = 0; i < requiredEdges.size(); ++i){
        const ReqEdge& e = requiredEdges[i];
        out << "    {\"line\": " << e.lineId
            << ", \"u\": " << e.u
            << ", \"v\": " << e.v
            << ", \"de\": " << e.de << "}";
        out << (i + 1 < requiredEdges.size() ? ",\n" : "\n");
    }
    out << "  ],\n";

    out << "  \"arcs\": [\n";
    for(size_t i = 0; i < arcs.size(); ++i){
        const Arc& a = arcs[i];
        out << "    {\"i\": " << a.i
            << ", \"j\": " << a.j
            << ", \"dij\": " << a.dij << "}";
        out << (i + 1 < arcs.size() ? ",\n" : "\n");
    }
    out << "  ],\n";

    out << "  \"parameters\": {\n";
    out << "    \"vd\": null,\n";
    out << "    \"_vd_preencher\": \"velocidade de voo do drone em deadheading\",\n";
    out << "    \"vs\": null,\n";
    out << "    \"_vs_preencher\": \"velocidade de voo do drone durante o servico\",\n";
    out << "    \"vc\": null,\n";
    out << "    \"_vc_preencher\": \"velocidade media do caminhao\",\n";
    out << "    \"L\": null,\n";
    out << "    \"_L_preencher\": \"autonomia maxima de um voo de drone (tempo)\",\n";
    out << "    \"beta\": null,\n";
    out << "    \"_beta_preencher\": \"alcance maximo do drone\",\n";
    out << "    \"kappa_d\": {\n";
    for(size_t i = 0; i < depots.size(); ++i){
        out << "      \"" << depots[i] << "\": null";
        out << (i + 1 < depots.size() ? ",\n" : "\n");
    }
    out << "    },\n";
    out << "    \"_kappa_d_preencher\": \"tempo de set-up/set-down em cada ponto d in D\"\n";
    out << "  },\n";

    out << "  \"coordinates\": {\n";
    {
        size_t i = 0, n = coordinates.size();
        for(const auto& kv : coordinates){
            out << "    \"" << kv.first << "\": {\"x\": " << kv.second.x
                << ", \"y\": " << kv.second.y << "}";
            out << (++i < n ? ",\n" : "\n");
        }
    }
    out << "  }\n";

    out << "}\n";

    out.close();
}