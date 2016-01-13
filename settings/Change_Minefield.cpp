//#include <QCoreApplication>
#include <stdio.h>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <math.h>

#include <vector>

#include <stdlib.h>     /* atof */
#include <time.h>

using namespace std;

string to_str(int number) {
    stringstream ss;
    ss << number;
    string str = ss.str();
    return str;
}

string to_str(double number) {
    stringstream ss;
    ss << number;
    string str = ss.str();
    return str;
}

int main(int argc, char *argv[])
{
    // QCoreApplication a(argc, argv);

    char Str[10000];
    FILE *arqIn, *arqOut, *arq;
    char result;
    
//    ./main    <Minefield>      <Cont_Mines>    <percentual de minhas na trilha>
//    argv[0]    argv[1]            argv[2]                    argv[3]

    string from = argv[1]; 
    string to = "minefield.yaml"; 
    from = "minefield/"+from+"minefield.yaml";

    //copy files "minefields".
    std::ifstream src(from.c_str(), std::ios::binary);
    std::ofstream dest(to.c_str(), std::ios::binary);
    dest << src.rdbuf();


    //copiar cena diferente para o gazebo
    from = argv[1]; 
    to = "../Media/models/scenario.dae";// em 
    from = "../Media/models/scenarioDAE/"+from+"scenario.dae";//hratc2016_workspace/src/hratc2016_framework/Media/models/scenarioDAE

    //copy files "minefields".
    std::ifstream srcScenario(from.c_str(), std::ios::binary);
    std::ofstream destScenario(to.c_str(), std::ios::binary);
    destScenario << srcScenario.rdbuf();


    
    //-----------------------------------
    //Opening files of the mines.
    string adress = argv[1];
    adress = "minefield/"+adress+"in.txt";
    arqIn = fopen(adress.c_str(), "rt");

    if (arqIn == NULL)  // Se houve erro na abertura
    {
        printf("Problems to open file %s\n",adress.c_str());
    }
    adress = argv[1];
    adress = "minefield/" + adress + "out.txt";
    arqOut = fopen(adress.c_str(), "rt");
    if (arqOut == NULL)  // Se houve erro na abertura
    {
        printf("Problems to open file %s\n",adress.c_str());
    }


//    float percentualMinasTrilha= float(atoi(argv[3]));	
//    int value_in = int(atoi(argv[2]) * (percentualMinasTrilha/100.));
//    int value_out = atoi(argv[2]) - value_in;
    int value_in = 3;
    int value_out = 2;


    vector<float> x_in, x_out;
    vector<float> y_in, y_out;
    float cx, cy;

    //reading mine files
    while(fscanf(arqIn,"%f %f", &cx, &cy) != EOF) //NUMERO DE LINHAS DO ARQUIVO
    {
        x_in.push_back(cx);
        y_in.push_back(cy);
    }
    fclose(arqIn);
    while(fscanf(arqOut,"%f %f", &cx, &cy) != EOF) //NUMERO DE LINHAS DO ARQUIVO
    {
        x_out.push_back(cx);
        y_out.push_back(cy);
    }
    fclose(arqOut);

    arq = fopen("judge.yaml", "wt");  // Cria um arquivo texto para gravação    
    if (arq == NULL) // Se não conseguiu criar
    {
        printf("Problems with the creation of file judge.yaml\n");
        //return;
    }


    srand(time(NULL));
    vector<double> Xpoints;
    vector<double> Ypoints;
    vector<float> copy_x_in = x_in;
    vector<float> copy_x_out = x_out;
    vector<float> copy_y_in = y_in;
    vector<float> copy_y_out = y_out;
    int checkCoordDist=0;
    int i;
    int inTrail;
    int limit;
    if(atoi(argv[1]) == 2){
        limit = 4;
    }else{
        limit = 5; 
    }
    do{
        Xpoints.clear();
        Ypoints.clear();
        x_in = copy_x_in;
        x_out = copy_x_out;
        y_in = copy_y_in;
        y_out = copy_y_out;

        for(i = 0; i < value_in; i++){
            if(x_in.size()>0){
                int pos=rand() % x_in.size();            
                double xi=x_in.at(pos);x_in.erase(x_in.begin()+pos);
                double yi=y_in.at(pos);y_in.erase(y_in.begin()+pos);
        	    if(Xpoints.size()==0){
                    Xpoints.push_back(xi);
        		    Ypoints.push_back(yi);
        	    }
                else{
                    for(int c=0; c<Xpoints.size();c++){
                        //double dist=sqrt(pow(Xpoints.at(c)-xi,2)+pow(Ypoints.at(c)-yi,2));
        			    double distX=sqrt(pow((Xpoints.at(c)-xi),2));
                        double distY=sqrt(pow((Ypoints.at(c)-yi),2));
        			    if(distX<1.5 || distY<1.5){
                            checkCoordDist=1;
                        }
        		    }
        		    if(checkCoordDist==0){
        			    Xpoints.push_back(xi);
        			    Ypoints.push_back(yi);
        		    }else{
        			    checkCoordDist=0;
        			    i=i-1;
        		    }
        	    }
            }
        }

        inTrail=Xpoints.size();

        for(int j = 0; j < value_out+(value_in-inTrail); j++){
            if(x_out.size()>0){
                int pos=rand() % x_out.size();
                double xi=x_out.at(pos);x_out.erase(x_out.begin()+pos);
                double yi=y_out.at(pos);y_out.erase(y_out.begin()+pos);

    	        if(Xpoints.size()==0){
                    Xpoints.push_back(xi);
                    Ypoints.push_back(yi);
                }else{
                    for(int c=0; c<Xpoints.size();c++){
                        //double dist=sqrt(pow(Xpoints.at(c)-xi,2)+pow(Ypoints.at(c)-yi,2));
                        double distX=sqrt(pow(Xpoints.at(c)-xi,2));
                        double distY=sqrt(pow(Ypoints.at(c)-yi,2));
                        if(distX<1.5 || distY<1.5){
                            checkCoordDist=1;
                            //cout<<xi<<" "<<yi<<endl;
                        }
                    }
                    if(checkCoordDist==0){
                        Xpoints.push_back(xi);
                        Ypoints.push_back(yi);
                    }else{
                        checkCoordDist=0;
                        j=j-1;
                    }
                }
            }
        }
    }while(Xpoints.size() < limit);

    strcpy(Str,"judge: \n\n  map_resolution: 0.05\n\n  random_mines: false\n\n  detection_min_dist: 0.5\n\n  explosion_max_dist: 0.3\n\n  num_mines: ");
    result = fputs(Str, arq);
    if (result == EOF)
        printf("ERROR\n");
    int numM = Xpoints.size();
    strcpy(Str,to_str(numM).c_str());
    result = fputs(Str, arq);
    if (result == EOF)
        printf("ERROR\n");
    strcpy(Str,"\n\n  mines_positions:");
    result = fputs(Str, arq);
    if (result == EOF)
        printf("ERROR\n");


    for(int i=0;i<Xpoints.size();i++){
        strcpy(Str, "\n\n    mine");
        fputs(Str, arq);
        strcpy(Str, to_str(i+1).c_str());
        fputs(Str, arq);
        strcpy(Str, ": \n      x: ");
        fputs(Str, arq);
        strcpy(Str, to_str(Xpoints.at(i)).c_str());
        fputs(Str, arq);
        strcpy(Str, "\n      y: ");
        fputs(Str, arq);
        strcpy(Str, to_str(Ypoints.at(i)).c_str());
        fputs(Str, arq);
    }

    cout<<inTrail<<" trail mines, "<<(Xpoints.size()-inTrail)<<" mines out of the trail"<<endl;

    fclose(arq);

     return 0;
}
