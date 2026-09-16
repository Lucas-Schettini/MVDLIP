#include "Data.h"

int main(int argc, char** argv){

    Data data = Data(argv[1]);
    data.exportJSON("teste.json");

    return 0;
}