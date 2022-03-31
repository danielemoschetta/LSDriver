#include "LaserScannerDriver.h"
#include <vector>
#include <math.h>

//Costruttore con risoluzione angolare in input
LaserScannerDriver::LaserScannerDriver(const double& r){
	if(r >= 0.1 && r <= 1.0){
		ris=r;
		scan_length=trunc(VIEW_ANGLE / ris) + 1;
		scan=new double*[BUFFER_DIM];
		for(int i=0;i<BUFFER_DIM;i++)
			scan[i]=nullptr;
	}
	else
		throw InvalidResolutionException();
}

//Distruttore
LaserScannerDriver::~LaserScannerDriver(){
	clear_buffer();
	delete[] scan;
	scan=nullptr;
}

//Costruttore di copia
LaserScannerDriver::LaserScannerDriver(const LaserScannerDriver& other)
	: ris(other.ris), scan_length(other.scan_length), head(other.head), tail(other.tail), full(other.full) 
{
	scan=new double*[BUFFER_DIM];
	for(int i=0;i<BUFFER_DIM;i++){
		if(other.scan[i]){
			scan[i]=new double[scan_length];
			std::copy(other.scan[i],other.scan[i] + scan_length, scan[i]);
		}
		else
			scan[i]=nullptr;
	}
}

//Assegnamento di copia
LaserScannerDriver& LaserScannerDriver::operator=(const LaserScannerDriver& other){
	if(this != &other){									//controllo sull'auto-assegnamento
		ris=other.ris;
		scan_length=other.scan_length;
		head=other.head;
		tail=other.tail;
		full=other.full;
		
		double** temp=new double*[BUFFER_DIM];			//viene copiato il buffer in un puntatore temporaneo		
		for(int i=0;i<BUFFER_DIM;i++){					//(per non deallocare il buffer di destinazione prima 
			if(other.scan[i]){							// di sapere se la copia e' andata a buon fine)
				temp[i]=new double[scan_length];
				std::copy(other.scan[i],other.scan[i] + scan_length, temp[i]);
			}
			else
				temp[i]=nullptr;
		}
		delete[] scan;									//la copia e' riuscita, ora viene deallocato
		scan=temp;
	}
	return *this;
}

//Costruttore di spostamento, noexcept per evitare la perdita di dati nel caso venga lanciata un'eccezione nello spostamento 
LaserScannerDriver::LaserScannerDriver(LaserScannerDriver&& other) noexcept
	: scan(other.scan), ris(other.ris), scan_length(other.scan_length), head(other.head), tail(other.tail), full(other.full) 
{
	other.head=0;						//l'oggetto di origine viene ripristinato
	other.tail=0;
	other.full=false;
	other.scan=nullptr;					//per evitare che una futura operazione sull'oggetto 
										//di orgine causi problemi a quello di destinazione
}

//Assegnamento di spostamento, noexcept per evitare la perdita di dati nel caso venga lanciata un'eccezione nello spostamento 
LaserScannerDriver& LaserScannerDriver::operator=(LaserScannerDriver&& other) noexcept{
	if(this != &other){						//controllo sull'auto-assegnamento
		clear_buffer();						//l'oggetto destinazione viene svuotato
		delete[] scan;
		
		scan=other.scan;					//i dati vengono copiati
		ris=other.ris;
		scan_length=other.scan_length;
		head=other.head;
		tail=other.tail;
		full=other.full;
		
		other.head=0;						//l'oggetto di origine viene ripristinato
		other.tail=0;
		other.full=false;
		other.scan=nullptr;					//per evitare che una futura operazione sull'oggetto 
											//di orgine causi problemi a quello di destinazione
	}
	return *this;	
}

//Inserimento nel buffer di una scansione, vector<double> in input
void LaserScannerDriver::new_scan(const std::vector<double> &v){
	if(is_full()){									//se il buffer è pieno viene rimosso il meno recente
		get_scan();
	}
	
	scan[tail]=new double[scan_length];	
	for(int i=0;i<scan_length;i++){
		if(i<v.size())								//finche' ci sono elementi nel vettore vengono inseriti
			scan[tail][i]=v.at(i);
		else										//se finiscono il resto viene riempito a 0
			scan[tail][i]=0.0;						
	}
	
	tail=(tail + 1) % BUFFER_DIM;
	full=(head==tail);								//se l'ultimo inserimento ha riempito il buffer, full diventa true
}

//Rimozione e restituzione della scansione meno recente, ritorna vector<double>
std::vector<double> LaserScannerDriver::get_scan(){
	if(is_empty())
		throw EmptyBufferException();

	std::vector<double> oldest;						//vengono salvati i dati del meno recente, da restituire
	for(int i=0;i<scan_length;i++)
		oldest.push_back(scan[head][i]);
	delete[] scan[head];							//la memoria della scansione meno recente viene deallocata
	scan[head]=nullptr;
	
	head=(head + 1) % BUFFER_DIM;					//la testa viene riposizionata al nuovo meno recente
	full=false;										//dopo aver rimosso un elemento sicuramente il buffer non sara' pieno
	return oldest;
}

//Svuotamento del buffer eliminando tutte le scansioni
void LaserScannerDriver::clear_buffer(){
	for(int i=0;i<BUFFER_DIM;i++){
		if(scan[i]){								//prima di deallocare si controlla se la scansione fosse allocata, 
			delete[] scan[i];						//così da evitare un doppio delete
			scan[i]=nullptr;
		}
	}
	head=0;											//gli indici vengono ripristinati alla situazione iniziale
	tail=0;
	full=false;										//dopo aver svuotato il buffer, sicuramente non sara' pieno
}

//Restituzione della lettura dell'angolo piu' vicino a quello dato nella scansione piu' recente
double LaserScannerDriver::get_distance(const double& a) const{
	if(is_empty())
		throw EmptyBufferException();
	
	double angle=fabs(fmod(a,360));					//l'angolo viene semplificato (es. 10=370=730=-350=ecc..)
	double min=fabs(angle-360); int closest=0;		//si parte dall'angolo giro per poi controllare 
	for(int i=0;i<scan_length;i++){					//quale sia l'angolo piu' vicino a quello dato					
		if(fabs(angle-i*ris) < min){
			min=fabs(angle-i*ris);
			closest=i;
		}
	}
	
	int newest_pos=((tail - 1) + BUFFER_DIM) % BUFFER_DIM;					//calcolo la posizione dell'ultima scansione inserita
	return scan[newest_pos][closest];										//restiuisco la misura piu' recente
}

//Overload dell'operator<<
std::ostream& operator<<(std::ostream& os, LaserScannerDriver lsd)
{
	if(lsd.is_empty())														//in caso di buffer vuoto, stampa un errore 
		return os<<"The buffer is empty.";									//(eccessivo lanciare un'eccezione)
	
	std::vector<double> to_print=lsd.newest_scan();
	for(int i=0;i<to_print.size();i++)
		os << "(" << i*lsd.getRis() << "," << to_print.at(i) << ")";		//viene calcolato l'angolo in base all'indice
    return os<<"\n";														//e stampato insieme alla distanza relativa
}

//Restituzione della scansione piu' recente, ritorna vector<double>
std::vector<double> LaserScannerDriver::newest_scan() const{
	if(is_empty())
		throw EmptyBufferException();
	
	std::vector<double> newest;
	int newest_pos=((tail - 1) + BUFFER_DIM) % BUFFER_DIM;			//viene calcolata la posizione dell'ultima scansione inserita
	for(int i=0;i<scan_length;i++)
		newest.push_back(scan[newest_pos][i]);
	return newest;
}

bool LaserScannerDriver::is_empty() const{
	return (!full && (head == tail));				//se i due indici puntano alla stessa cella e full==false, il buffer e' vuoto
}													//altrimenti e' pieno

bool LaserScannerDriver::is_full() const{
	return full;
}
