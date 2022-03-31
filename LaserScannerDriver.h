#ifndef LASERSCANNERDRIVER_H
#define LASERSCANNERDRIVER_H

#include <iostream>
#include <vector>
/*
	LaserScannerDriver
	Questa classe gestisce un buffer di scansioni effettuate da un LIDAR, ciascuna contenente letture di distanze.
	Il buffer è gestito in modo circolare (https://en.wikipedia.org/wiki/Circular_buffer) utilizzando: 
				- puntatore all'indirizzo del buffer nel free store (scan);
		   		- indice della cella della scansione meno recente (head);
		   		- indice della cella in cui inserire la prossima scansione (tail);
				- capienza massima del buffer (BUFFER_DIM);
				- flag per gestire la sovrapposizone degli indici (full).
	
	Invarianti: - 0.1 <= ris <= 1.0 ;
				- 0 <= head < BUFFER_DIM ;
				- 0 <= tail < BUFFER_DIM .
				- scan == nullptr oppure allocato con un array nel free store di dimensione BUFFER_DIM
	
	Non e' previsto un costruttore senza parametri, in quanto la risoluzione angolare varia in base al LIDAR 
	ed e' un dato imprescindibile per la gestione del relativo buffer di scansioni.
*/

class LaserScannerDriver{
	public:
		//costruttore e distruttore
		LaserScannerDriver(const double&);
		~LaserScannerDriver();
		
		//copia/assegnamento necessari per "the rule of three/five/zero" (https://en.cppreference.com/w/cpp/language/rule_of_three)
		LaserScannerDriver(const LaserScannerDriver&);
		LaserScannerDriver& operator=(const LaserScannerDriver&);
		LaserScannerDriver(LaserScannerDriver&& other) noexcept;
		LaserScannerDriver& operator=(LaserScannerDriver&& other) noexcept;
		
		//funzioni richieste
		void new_scan(const std::vector<double>&);
		std::vector<double> get_scan();
		void clear_buffer();
		double get_distance(const double&) const; 
		
		//funzioni necessarie/utili
		double getRis() const { return ris; }
		std::vector<double> newest_scan() const;
		bool is_empty() const;
		bool is_full() const;
		
		//eccezioni
		class EmptyBufferException {};
		class InvalidResolutionException {};
		
	private:
		static constexpr int BUFFER_DIM=10;			//numero di elementi che il buffer deve gestire
		static constexpr double VIEW_ANGLE=180;		//angolo di visione del LIDAR
		
		double **scan=nullptr;						//puntatore al buffer
		double ris;									//risoluzione angolare
		int scan_length;							//numero di rilevazioni contenute in ogni scansione
		int head=0;									//testa del buffer circolare
		int tail=0;									//coda del buffer circolare
		bool full=false;
};

std::ostream& operator<<(std::ostream& os, LaserScannerDriver lsd);

#endif // LASERSCANNERDRIVER_H
