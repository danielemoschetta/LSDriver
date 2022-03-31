#include "LaserScannerDriver.h"
#include <math.h>
#include <stdlib.h>
#include <time.h>

int main(){
	srand(time(0));
	double ris=1, scan_length=trunc(180/ris)+1;
	
	std::cout<<"\nProvo ad inizializzare un LaserScannerDriver con risoluzione non valida:\n";
	try{
		LaserScannerDriver lsd(0);
	}catch(LaserScannerDriver::InvalidResolutionException){
		std::cout<<"InvalidResolutionException caught!\n";
	}
	LaserScannerDriver lsd(ris);
	
	std::cout<<"\nInserisco BUFFER_DIM + 1 scansioni di valori casuali (la prima verra' sovrascritta)\n";
	for(int i=0;i<11;i++){
		std::vector<double> v;
		for(int j=0;j<scan_length;j++)
			v.push_back(rand() % 100);
		lsd.new_scan(v);
	}
	
	std::cout<<"\nUltima scansione:\n"<<lsd<<std::endl;
	std::cout<<"Provo a richiedere l'ultima lettura di alcuni angoli:\n";
	std::cout<<"Angolo: 1 -> Lettura: "<<lsd.get_distance(1)<<std::endl;
	std::cout<<"Angolo: 70.4 -> Lettura: "<<lsd.get_distance(70.4)<<std::endl;
	std::cout<<"Angolo: 70.6 -> Lettura: "<<lsd.get_distance(70.6)<<std::endl;
	std::cout<<"Angolo: 181 -> Lettura: "<<lsd.get_distance(181)<<std::endl;
	std::cout<<"Angolo: 269 -> Lettura: "<<lsd.get_distance(269)<<std::endl;
	std::cout<<"Angolo: 350.7 -> Lettura: "<<lsd.get_distance(350.7)<<std::endl;
	std::cout<<"Angolo: 721.3 -> Lettura: "<<lsd.get_distance(721.3)<<std::endl;
	
	std::cout<<"\nProvo ad inserire un vettore vuoto e lo stampo:\n";
	std::vector<double> v;
	lsd.new_scan(v);
	std::cout<<lsd;
	
	std::cout<<"\nProvo a copiare il buffer in un altro e a svuotare il primo.\n";
	LaserScannerDriver nuovo(lsd);
	lsd.clear_buffer();
	
	std::cout<<"\nProvo a farmi resituire la scansione meno recente dal buffer appena svuotato:\n";
	try{
		std::vector<double> print;
		print=lsd.get_scan();
		for(double x : print)
			std::cout<<x<<" ";
		std::cout<<std::endl;
	}catch(LaserScannerDriver::EmptyBufferException){
		std::cout<<"EmptyBufferException caught!\n";
	}
	
	std::cout<<"\nProvo a richiedere l'ultima scansione dal buffer dove avevo copiato il precedente:\n";
	std::cout<<nuovo;
	return 0;
}
