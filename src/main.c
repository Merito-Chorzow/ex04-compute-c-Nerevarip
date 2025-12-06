#include <stdio.h>
#include "q15.h"
#include "pid.h"
#include "plant.h"

int main(void){
    // Konfiguracja #1 (Dostosuj te wartości w kolejnych testach!)
    pid_t pid = {
        .kp = f2q15(0.3f),
        .ki = f2q15(0.01f),
        .kd = f2q15(0.2f),   // Zwiększyłem trochę Kd, żeby było widać efekt
        .i_acc = 0,
        .d_prev = 0,
        .d_filt_prev = 0,    // <--- INICJALIZACJA NOWEGO POLA
        .d_alpha = f2q15(0.85f),
        .i_limit = f2q15(0.5f),
        .u_min = f2q15(-1.0f),
        .u_max = f2q15( 1.0f),
    };

    q15 y = f2q15(0.0f);     // Stan początkowy obiektu
    q15 set = f2q15(0.5f);   // Wartość zadana
    q15 alpha = f2q15(0.05f);// Parametr bezwładności obiektu

    printf("Start simulation\n");
    // Nagłówek dla łatwiejszego kopiowania do Excela/analizy
    printf("k, set, y, u\n");

    for(int k=0; k<1000; k++){
        q15 u = pid_step(&pid, set, y);
        y = plant_step(y, u, alpha);

        // Logowanie co 50 kroków (zgodnie z instrukcją)
        if(k % 50 == 0){
             // %.3f wypisuje floaty, które konwertujemy z q15
            printf("%d, %.3f, %.3f, %.3f\n",
                k, q15tof(set), q15tof(y), q15tof(u));
        }
    }
    return 0;
}