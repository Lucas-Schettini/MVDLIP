#include "Data.h"

int main(int argc, char** argv){

    Data data = Data(argv[1]);
    data.setDepot(0, 150.0, 150.0);   
    data.buildGroundArcs();           
    data.exportJSON("teste.json");

    return 0;
}