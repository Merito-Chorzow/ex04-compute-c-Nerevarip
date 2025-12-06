#include "pid.h"

q15 pid_step(pid_t* pid, q15 set, q15 meas){
    // 1. Oblicz uchyb (błąd)
    q15 e = set - meas;

    // --- CZŁON P ---
    // Mnożenie q15 * q15 daje wynik Q30, musimy przesunąć o 15 w prawo
    int32_t p_raw = (int32_t)pid->kp * (int32_t)e; 
    q15 p_term = (q15)(p_raw >> 15);

    // --- CZŁON I (z Anti-Windup typu Clamping) ---
    // Liczymy nową całkę w szerszym typie int32, żeby wykryć przepełnienie
    int32_t i_new = (int32_t)pid->i_acc + (((int32_t)pid->ki * (int32_t)e) >> 15);
    
    // Clamping (ograniczenie)
    if (i_new >  pid->i_limit) i_new =  pid->i_limit;
    if (i_new < -pid->i_limit) i_new = -pid->i_limit;
    pid->i_acc = (q15)i_new;

    // --- CZŁON D (z filtrem LPF) ---
    // Różniczka: de = e - e_prev
    q15 de = e - pid->d_prev;
    
    // Filtr LPF: D[k] = alpha * D[k-1] + (1-alpha) * Kd * de
    // Uwaga: Kd zazwyczaj mnożymy tutaj lub przy sumowaniu. 
    // Przyjmijmy wersję z instrukcji: d_filt działa na samej różnicy, a Kd jest wzmocnieniem.
    // Ale w Twoim kodzie Kd nie było użyte w D! Naprawmy to mnożąc 'de' przez 'Kd'.
    
    q15 de_scaled = mul_q15(pid->kd, de); // Wzmocnienie różniczki

    q15 one_minus_alpha = (q15)(32767 - pid->d_alpha);
    
    // Wzór filtru: y[k] = alpha * y[k-1] + (1-alpha) * x[k]
    q15 d_term = (q15)( ((int32_t)pid->d_alpha * (int32_t)pid->d_filt_prev >> 15)
                      + ((int32_t)one_minus_alpha * (int32_t)de_scaled >> 15) );
    
    // Zapisz stan na następny krok
    pid->d_prev = e;           // zapisz błąd
    pid->d_filt_prev = d_term; // zapisz stan filtru

    // --- SUMA I SATURACJA ---
    int32_t sum = (int32_t)p_term + (int32_t)pid->i_acc + (int32_t)d_term;
    
    if (sum > pid->u_max) sum = pid->u_max;
    if (sum < pid->u_min) sum = pid->u_min;

    return (q15)sum;
}