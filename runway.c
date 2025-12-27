//Bryan Funes
//ID:1002103222
/* Copyright (c) 2025 Trevor Bakker
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILTY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/license/>.
*/

#define _GNU_SOURCE

#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <assert.h>
#include <time.h>

/*** Constants that define parameters of the simulation ***/

#define MAX_RUNWAY_CAPACITY 2    /* Number of aircraft that can use runway simultaneously */
#define CONTROLLER_LIMIT 8       /* Number of aircraft the controller can manage before break */
#define MAX_AIRCRAFT 1000        /* Maximum number of aircraft in the simulation */
#define FUEL_MIN 20              /* Minimum fuel reserve in seconds */
#define FUEL_MAX 60              /* Maximum fuel reserve in seconds */
#define EMERGENCY_TIMEOUT 30     /* Max wait time for emergency aircraft in seconds */
#define DIRECTION_SWITCH_TIME 5  /* Time required to switch runway direction */
#define DIRECTION_LIMIT 3        /* Max consecutive aircraft in same direction */

#define COMMERCIAL 0
#define CARGO 1
#define EMERGENCY 2

#define NORTH 0
#define SOUTH 1
#define EAST  2
#define WEST  4

/* TODO */
/* Add your synchronization variables here */

/* basic information about simulation.  they are printed/checked at the end 
* and in assert statements during execution.
*
* you are responsible for maintaining the integrity of these variables in the 
* code that you develop. 
*/
pthread_mutex_t Mutex_ENTER = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t Mutex_COM = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t Mutex_CAR = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t Mutex_EMER = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t Mutex_COML = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t Mutex_CARL = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t Mutex_EMERL = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t Mutex_WAITCOM = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t Mutex_WAITCAR = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t Mutex_FUEL_CAR = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t Mutex_FUEL_COM = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t Cond_COM = PTHREAD_COND_INITIALIZER;
pthread_cond_t Cond_CAR = PTHREAD_COND_INITIALIZER;
pthread_cond_t Cond_EMER = PTHREAD_COND_INITIALIZER;
pthread_cond_t Cond_FUEL_CAR = PTHREAD_COND_INITIALIZER;
pthread_cond_t Cond_FUEL_COM = PTHREAD_COND_INITIALIZER;

int ready_COM;
int ready_CAR;
int ready_EMER;
int ready_FUEL_COM;
int ready_FUEL_CAR;
int Commercial_Waiting;
int Cargo_Waiting;
int Emergency_Waiting;
int Fuel_Waiting_COM;
int Fuel_Waiting_CAR;
int CAR_consecutive;
int COM_consecutive;
int Signal_COM;
int Signal_CAR;
int Fuel_COM;
int Fuel_CAR;
int Priority_COM;
int Priority_CAR;
int CAR_ticket;
int COM_ticket;

static int aircraft_on_runway = 0;       /* Total number of aircraft currently on runway */
static int commercial_on_runway = 0;     /* Total number of commercial aircraft on runway */
static int cargo_on_runway = 0;          /* Total number of cargo aircraft on runway */
static int emergency_on_runway = 0;      /* Total number of emergency aircraft on runway */
static int aircraft_since_break = 0;     /* Aircraft processed since last controller break */
static int current_direction = NORTH;    /* Current runway direction (NORTH or SOUTH) */
static int consecutive_direction = 0;    /* Consecutive aircraft in current direction */


typedef struct 
{
  int arrival_time;         // time between the arrival of this aircraft and the previous aircraft
  int runway_time;          // time the aircraft needs to spend on the runway
  int aircraft_id;
  int aircraft_type;        // COMMERCIAL, CARGO, or EMERGENCY
  int fuel_reserve;         // Randomly assigned fuel reserve (FUEL_MIN to FUEL_MAX seconds)
  time_t arrival_timestamp; // timestamp when aircraft thread was created
} aircraft_info;

/* Called at beginning of simulation.  
 * TODO: Create/initialize all synchronization
 * variables and other global variables that you add.
 */
static int initialize(aircraft_info *ai, char *filename) 
{
  aircraft_on_runway    = 0;
  commercial_on_runway  = 0;
  cargo_on_runway       = 0;
  emergency_on_runway   = 0;
  aircraft_since_break  = 0;
  current_direction     = NORTH;
  consecutive_direction = 0;

  /* Initialize your synchronization variables (and 
   * other variables you might use) here
   */

  ready_CAR = 0;
  ready_COM = 0;
  ready_EMER = 0;
  ready_FUEL_COM = 0;
  ready_FUEL_CAR = 0;
  Commercial_Waiting = 0;
  Cargo_Waiting = 0;
  Emergency_Waiting = 0;
  Fuel_Waiting_CAR = 0;
  Fuel_Waiting_COM = 0;
  CAR_consecutive = 0;
  COM_consecutive = 0;
  Signal_CAR = 0;
  Signal_COM = 0;
  Fuel_COM = 0;
  Fuel_CAR = 0;
  Priority_COM = 2;
  Priority_CAR = 2;

  /* seed random number generator for fuel reserves */
  srand(time(NULL));

  /* Read in the data file and initialize the aircraft array */
  FILE *fp;

  if((fp=fopen(filename, "r")) == NULL) 
  {
    printf("Cannot open input file %s for reading.\n", filename);
    exit(1);
  }

  int i = 0;
  char line[256];
  while (fgets(line, sizeof(line), fp) && i < MAX_AIRCRAFT) 
  {
    /* Skip comment lines and empty lines */
    if (line[0] == '#' || line[0] == '\n' || line[0] == '\r') {
      continue;
    }
    
    /* Parse the line */
    if (sscanf(line, "%d%d%d", &(ai[i].aircraft_type), &(ai[i].arrival_time), 
               &(ai[i].runway_time)) == 3) {
      /* Assign random fuel reserve between FUEL_MIN and FUEL_MAX */
      ai[i].fuel_reserve = FUEL_MIN + (rand() % (FUEL_MAX - FUEL_MIN + 1));
      i = i + 1;
    }
  }

  fclose(fp);
  return i;
}

/* Code executed by controller to simulate taking a break 
 * You do not need to add anything here.  
 */
__attribute__((unused)) static void take_break() 
{
  printf("The air traffic controller is taking a break now.\n");
  sleep(5);
  assert( aircraft_on_runway == 0 );
  aircraft_since_break = 0;
}

/* Code executed to switch runway direction
 * You do not need to add anything here.
 */
__attribute__((unused)) static void switch_direction()
{
  printf("Switching runway direction from %s to %s\n",
         current_direction == NORTH ? "NORTH" : "SOUTH",
         current_direction == NORTH ? "SOUTH" : "NORTH");
  
  assert( aircraft_on_runway == 0 );  // Runway must be empty to switch
  
  sleep(DIRECTION_SWITCH_TIME);
  
  current_direction = (current_direction == NORTH) ? SOUTH : NORTH;
  consecutive_direction = 0;
  
  printf("Runway direction switched to %s\n",
         current_direction == NORTH ? "NORTH" : "SOUTH");
}

/* Code for the air traffic controller thread. This is fully implemented except for 
 * synchronization with the aircraft. See the comments within the function for details.
 *Function: controller_thread
 *Parameters: Controller_Thread_info - pointer to aircraft information structure
 *Returns: void
 *Description: This controls which aircrafts can go on to the runway.
 *             It checks to make sure that not too many are on the runway
 *             and checks for consecutiveness and for the controller to 
 *             take a break. When a aircraft can go on the runway, the aircraft gets signaled.
 */
void *controller_thread(void *arg) 
{
  // Suppress the warning for now
 (void)arg;

  printf("The air traffic controller arrived and is beginning operations\n");

  /* Loop while waiting for aircraft to arrive. */
  while (1) 
  {
    /* TODO */
    /* Add code here to handle aircraft requests, controller breaks,      */
    /* and runway direction switches.                                     */
    /* Currently the body of the loop is empty.  There's no communication */
    /* between controller and aircraft, i.e. all aircraft are admitted    */
    /* without regard for runway capacity, aircraft type, direction,      */
    /* priorities, and whether the controller needs a break.              */
    /* You need to add all of this.                                       */
    if(consecutive_direction > DIRECTION_LIMIT && aircraft_on_runway == 0)
    {
      if(current_direction == SOUTH && Commercial_Waiting > 0)
      {
        switch_direction();
      }
      else if(current_direction == NORTH && Cargo_Waiting > 0)
      {
        switch_direction();
      }
      else
      {
        consecutive_direction = 2;
      }
    }
    if(aircraft_since_break == CONTROLLER_LIMIT && aircraft_on_runway == 0)
    {
      take_break();
    }
    if(aircraft_on_runway < MAX_RUNWAY_CAPACITY && aircraft_since_break < CONTROLLER_LIMIT 
      && consecutive_direction <= DIRECTION_LIMIT)
    {
      if(Fuel_Waiting_CAR > 0 && commercial_on_runway == 0)
      {
        if(current_direction != SOUTH && aircraft_on_runway == 0 && Fuel_Waiting_COM == 0)
        {
          switch_direction();
        }
        if(current_direction == SOUTH)
        {
          pthread_mutex_lock(&Mutex_FUEL_CAR);
          ready_FUEL_CAR = 1;
          COM_consecutive = 0;
          CAR_consecutive = 0;
          pthread_cond_signal(&Cond_FUEL_CAR);
          pthread_mutex_unlock(&Mutex_FUEL_CAR);
        }
      }
      if(Fuel_Waiting_COM > 0 && cargo_on_runway == 0)
      {
        if(current_direction != NORTH && aircraft_on_runway == 0 && Fuel_Waiting_CAR == 0)
        {
          switch_direction();
        }
        if(current_direction == NORTH)
        {
          pthread_mutex_lock(&Mutex_FUEL_COM);
          ready_FUEL_COM = 1;
          COM_consecutive = 0;
          CAR_consecutive = 0;
          pthread_cond_signal(&Cond_FUEL_COM);
          pthread_mutex_unlock(&Mutex_FUEL_COM);
        }
      }
      if(COM_consecutive == 4 && Fuel_Waiting_CAR == 0 && Fuel_Waiting_COM == 0)
      {
        if(current_direction != SOUTH && aircraft_on_runway == 0 && Commercial_Waiting == 0)
        {
          switch_direction();
        }
        if(current_direction == SOUTH)
        {
          pthread_mutex_lock(&Mutex_CAR);
          Signal_CAR = 1;
          COM_consecutive = 0;
          CAR_consecutive++;
          ready_CAR = 1;
          pthread_cond_signal(&Cond_CAR);
          pthread_mutex_unlock(&Mutex_CAR);
        }
      }
      if(CAR_consecutive == 4 && Fuel_Waiting_CAR == 0 && Fuel_Waiting_COM == 0)
      {
        if(current_direction != NORTH && aircraft_on_runway == 0 && Cargo_Waiting == 0)
        {
          switch_direction();
        }
        if(current_direction == NORTH)
        {
          pthread_mutex_lock(&Mutex_COM);
          Signal_COM = 1;
          COM_consecutive++;
          CAR_consecutive = 0;
          ready_COM = 1;
          pthread_cond_signal(&Cond_COM);
          pthread_mutex_unlock(&Mutex_COM);
        }
      }
      if(Emergency_Waiting > 0 && Fuel_Waiting_CAR == 0 && Fuel_Waiting_COM == 0)
      {
        pthread_mutex_lock(&Mutex_EMER);
        ready_EMER = 1;
        COM_consecutive = 0;
        CAR_consecutive = 0;
        pthread_cond_signal(&Cond_EMER);
        pthread_mutex_unlock(&Mutex_EMER);
      }
      if(cargo_on_runway == 0 && commercial_on_runway >= 0 && Commercial_Waiting > 0 && Emergency_Waiting
         == 0 && Fuel_Waiting_CAR == 0 && Fuel_Waiting_COM == 0)
      {
        if(current_direction != NORTH && aircraft_on_runway == 0 && Cargo_Waiting == 0)
        {
          switch_direction();
        }
        if(current_direction == NORTH)
        {
          pthread_mutex_lock(&Mutex_COM);
          Signal_COM = 1;
          COM_consecutive++;
          CAR_consecutive = 0;
          ready_COM = 1;
          pthread_cond_signal(&Cond_COM);
          pthread_mutex_unlock(&Mutex_COM);
        }
      }
      if(commercial_on_runway == 0 && cargo_on_runway >= 0 && Cargo_Waiting > 0 && 
        Emergency_Waiting == 0 && Fuel_Waiting_CAR == 0 && Fuel_Waiting_COM == 0)
      {
        if(current_direction != SOUTH && aircraft_on_runway == 0 && Commercial_Waiting == 0)
        {
          switch_direction();
        }
        if(current_direction == SOUTH)
        {
          pthread_mutex_lock(&Mutex_CAR);
          Signal_CAR = 1;
          COM_consecutive = 0;
          CAR_consecutive++;
          ready_CAR = 1;
          pthread_cond_signal(&Cond_CAR);
          pthread_mutex_unlock(&Mutex_CAR);
        }
      }
    }

    
    /* Allow thread to be cancelled */
    pthread_testcancel();
    usleep(100000); // 100ms sleep to prevent busy waiting
  }
  pthread_exit(NULL);
}


/* Code executed by a commercial aircraft to enter the runway.
 * You have to implement this.  Do not delete the assert() statements,
 * but feel free to add your own.
 * Function: commercial_enter
 * Parameters: aircraft_info - pointer to aircraft information structure.
 * Returns: void
 * Description: This function handles the control of commercial aircraft. They
 *              stay in a loop to check their fuel and when signaled will wait to
 *              get on the runway when signaled by the controller. When the aircraft
 *              is out of fuel it will print out a message and try to get priority.
 */
void commercial_enter(aircraft_info *arg) 
{
  // Suppress the compiler warning
  (void)arg;

  /* TODO */
  /* Request permission to use the runway. You might also want to add      */
  /* synchronization for the simulation variables below.                   */
  /* Consider: runway capacity, direction (commercial prefer NORTH),       */
  /* controller breaks, fuel levels, emergency priorities, and fairness.   */
  /*  YOUR CODE HERE.                                                      */
  pthread_mutex_lock(&Mutex_ENTER);

  Commercial_Waiting++;

  pthread_mutex_unlock(&Mutex_ENTER);

  while(1)
  {
    pthread_mutex_lock(&Mutex_WAITCOM);
    Fuel_COM = (int)time(NULL) - (int)arg->arrival_timestamp;
    if(Fuel_COM >= arg->fuel_reserve)
    {
      printf("EMERGENCY: Commercial Aircraft %d has ran out of reserved fuel and will land imminently!\n"
        , arg->aircraft_id);
      Commercial_Waiting--;
      Fuel_Waiting_COM++;
      pthread_mutex_unlock(&Mutex_WAITCOM);
      pthread_mutex_lock(&Mutex_FUEL_COM);
      while(!ready_FUEL_COM)
      {
        pthread_cond_wait(&Cond_FUEL_COM, &Mutex_FUEL_COM);
      }
      aircraft_on_runway    = aircraft_on_runway + 1;
      aircraft_since_break  = aircraft_since_break + 1;
      commercial_on_runway  = commercial_on_runway + 1;
      consecutive_direction = consecutive_direction + 1;
      ready_FUEL_COM = 0;
      Priority_COM = 0;
      Fuel_Waiting_COM--;
      pthread_mutex_unlock(&Mutex_FUEL_COM);
      return;
    }
    if(Signal_COM == 1)
    {
      Signal_COM = 0;
      pthread_mutex_unlock(&Mutex_WAITCOM);
      break;
    }
    pthread_mutex_unlock(&Mutex_WAITCOM);
    sleep(1);
  }

  pthread_mutex_lock(&Mutex_COM);

  while(!ready_COM)
  {
    pthread_cond_wait(&Cond_COM, &Mutex_COM);
  } 
  aircraft_on_runway    = aircraft_on_runway + 1;
  aircraft_since_break  = aircraft_since_break + 1;
  commercial_on_runway  = commercial_on_runway + 1;
  consecutive_direction = consecutive_direction + 1;
  ready_COM = 0;
  Commercial_Waiting--;
  pthread_mutex_unlock(&Mutex_COM);
}

/* Code executed by a cargo aircraft to enter the runway.
 * You have to implement this.  Do not delete the assert() statements,
 * but feel free to add your own.
 * Function: cargo_enter
 * Parameters: aircraft_info _ pointer to aircraft information structure.
 * Returns: void
 * Description: This function handles the way cargo enters the runway. The
 *              aircraft stays in a loop to simulate it running out of fuel
 *              ,when it runs out it makes an emergency warning and tries
 *              to get priority. To get on the runway, it must be signaled
 *              from the controller thread to continue on.
 */
void cargo_enter(aircraft_info *ai) 
{
  (void)ai;

  /* TODO */
  /* Request permission to use the runway. You might also want to add      */
  /* synchronization for the simulation variables below.                   */
  /* Consider: runway capacity, direction (cargo prefer SOUTH),            */
  /* controller breaks, fuel levels, emergency priorities, and fairness.   */
  /*  YOUR CODE HERE.                                                      */ 
  pthread_mutex_lock(&Mutex_ENTER);

  Cargo_Waiting++;

  pthread_mutex_unlock(&Mutex_ENTER);

  while(1)
  {
    pthread_mutex_lock(&Mutex_WAITCAR);
    Fuel_CAR = (int)time(NULL) - (int)ai->arrival_timestamp;
    if(Fuel_CAR >= ai->fuel_reserve)
    {
      printf("EMERGENCY: Cargo Aircraft %d has ran out of reserved fuel and will land imminently!\n"
        , ai->aircraft_id);
      Cargo_Waiting--;
      Fuel_Waiting_CAR++;
      pthread_mutex_unlock(&Mutex_WAITCAR);
      pthread_mutex_lock(&Mutex_FUEL_CAR);
      while(!ready_FUEL_CAR)
      {
        pthread_cond_wait(&Cond_FUEL_CAR, &Mutex_FUEL_CAR);
      }
      aircraft_on_runway    = aircraft_on_runway + 1;
      aircraft_since_break  = aircraft_since_break + 1;
      cargo_on_runway       = cargo_on_runway + 1;
      consecutive_direction = consecutive_direction + 1;
      ready_FUEL_CAR = 0;
      Priority_CAR = 0;
      Fuel_Waiting_CAR--;
      pthread_mutex_unlock(&Mutex_FUEL_CAR);
      return;
    }
    if(Signal_CAR == 1)
    {
      Signal_CAR = 0;
      pthread_mutex_unlock(&Mutex_WAITCAR);
      break;
    }
    pthread_mutex_unlock(&Mutex_WAITCAR);
    sleep(1);
  }

  pthread_mutex_lock(&Mutex_CAR);

  while(!ready_CAR)
  {
    pthread_cond_wait(&Cond_CAR, &Mutex_CAR);
  }
  aircraft_on_runway    = aircraft_on_runway + 1;
  aircraft_since_break  = aircraft_since_break + 1;
  cargo_on_runway       = cargo_on_runway + 1;
  consecutive_direction = consecutive_direction + 1;
  ready_CAR = 0;
  Cargo_Waiting--;
  pthread_mutex_unlock(&Mutex_CAR);
}

/* Code executed by an emergency aircraft to enter the runway.
 * You have to implement this.  Do not delete the assert() statements,
 * but feel free to add your own.
 * Function: emergency_enter
 * Parameters:aircraft_info - pointer to aircraft information structure
 * Returns: void
 * Description: This functions controls the entrance of the incoming emergency
 *              aircraft. The aircraft waits here until the controller signals
 *              it to get on the runway and it's safe to use.
 */
void emergency_enter(aircraft_info *ai) 
{
  (void)ai;

  /* TODO */
  /* Request permission to use the runway. You might also want to add      */
  /* synchronization for the simulation variables below.                   */
  /* Emergency aircraft have priority and must be admitted within 30s,     */
  /* but still respect runway capacity and controller breaks.              */
  /* Emergency aircraft can use either direction.                          */
  /*  YOUR CODE HERE.                                                      */ 

  pthread_mutex_lock(&Mutex_EMER);

  Emergency_Waiting++;

  while(!ready_EMER)
  {
    pthread_cond_wait(&Cond_EMER, &Mutex_EMER);
  }
  aircraft_on_runway = aircraft_on_runway + 1;
  aircraft_since_break = aircraft_since_break + 1;
  emergency_on_runway = emergency_on_runway + 1;
  consecutive_direction = consecutive_direction + 1;
  ready_EMER = 0;
  Emergency_Waiting--;
  pthread_mutex_unlock(&Mutex_EMER);
}

/* Code executed by an aircraft to simulate the time spent on the runway
 * You do not need to add anything here.  
 */
static void use_runway(int t) 
{
  sleep(t);
}


/* Code executed by a commercial aircraft when leaving the runway.
 * You need to implement this.  Do not delete the assert() statements,
 * but feel free to add as many of your own as you like.
 * Function:commercial_leave
 * Parameters: none
 * Returns: void
 * Description: Handles when a aircraft leaves. Just a simple
 *              mutex use and decrement of variables to get
 *              out of the runway.
 */
static void commercial_leave() 
{
  /* 
   *  TODO
   *  YOUR CODE HERE. 
   */
  pthread_mutex_lock(&Mutex_COML);
  aircraft_on_runway = aircraft_on_runway - 1;
  commercial_on_runway = commercial_on_runway - 1;
  pthread_mutex_unlock(&Mutex_COML);
}

/* Code executed by a cargo aircraft when leaving the runway.
 * You need to implement this.  Do not delete the assert() statements,
 * but feel free to add as many of your own as you like.
 * Function: cargo_leave
 * Parameters: none
 * Returns: void
 * Description: a simple leave function for the cargo aircraft.
 *              Uses a mutex to protect critical region to 
 *              decrement the variables to leave the runway.
 */
static void cargo_leave() 
{
  /* 
   * TODO
   * YOUR CODE HERE. 
   */
  pthread_mutex_lock(&Mutex_CARL);
  aircraft_on_runway = aircraft_on_runway - 1;
  cargo_on_runway = cargo_on_runway - 1;
  pthread_mutex_unlock(&Mutex_CARL);
}

/* Code executed by an emergency aircraft when leaving the runway.
 * You need to implement this.  Do not delete the assert() statements,
 * but feel free to add as many of your own as you like.
 * Function: emergency_leave
 * Parameters: none
 * Returns: void
 * Description: a simple leave function for the emergency aircraft.
 *              Uses a mutex to protect critical region to 
 *              decrement the variables to leave the runway.
 */
static void emergency_leave() 
{
  /* 
   * TODO
   * YOUR CODE HERE. 
   */
  pthread_mutex_lock(&Mutex_EMERL);
  aircraft_on_runway = aircraft_on_runway - 1;
  emergency_on_runway = emergency_on_runway - 1;
  pthread_mutex_unlock(&Mutex_EMERL);
}

/* Main code for commercial aircraft threads.  
 * You do not need to change anything here, but you can add
 * debug statements to help you during development/debugging.
 */
void* commercial_aircraft(void *ai_ptr) 
{
  aircraft_info *ai = (aircraft_info*)ai_ptr;
  
  /* Record arrival time for fuel tracking */
  ai->arrival_timestamp = time(NULL);

  /* Request runway access */
  commercial_enter(ai);

  printf("Commercial aircraft %d (fuel: %ds) is now on the runway (direction: %s)\n", 
         ai->aircraft_id, ai->fuel_reserve,
         current_direction == NORTH ? "NORTH" : "SOUTH");

  assert(aircraft_on_runway <= MAX_RUNWAY_CAPACITY && aircraft_on_runway >= 0);
  assert(commercial_on_runway >= 0 && commercial_on_runway <= MAX_RUNWAY_CAPACITY);
  assert(cargo_on_runway >= 0 && cargo_on_runway <= MAX_RUNWAY_CAPACITY);
  assert(emergency_on_runway >= 0 && emergency_on_runway <= MAX_RUNWAY_CAPACITY);
  assert(cargo_on_runway == 0 ); // Commercial and cargo cannot mix
  
  /* Use runway  --- do not make changes to the 3 lines below*/
  printf("Commercial aircraft %d begins runway operations for %d seconds\n", 
         ai->aircraft_id, ai->runway_time);
  use_runway(ai->runway_time);
  printf("Commercial aircraft %d completes runway operations and prepares to depart\n", 
         ai->aircraft_id);

  /* Leave runway */
  commercial_leave();  

  printf("Commercial aircraft %d has cleared the runway\n", ai->aircraft_id);

  if (!(aircraft_on_runway <= MAX_RUNWAY_CAPACITY && aircraft_on_runway >= 0)) {
    printf("ASSERT FAILURE: aircraft_on_runway=%d (should be 0-%d)\n", aircraft_on_runway, MAX_RUNWAY_CAPACITY);
    printf("Runway state: commercial=%d, cargo=%d, emergency=%d, direction=%s\n", 
           commercial_on_runway, cargo_on_runway, emergency_on_runway,
           current_direction == NORTH ? "NORTH" : "SOUTH");
  }
  assert(aircraft_on_runway <= MAX_RUNWAY_CAPACITY && aircraft_on_runway >= 0);
  assert(commercial_on_runway >= 0 && commercial_on_runway <= MAX_RUNWAY_CAPACITY);
  assert(cargo_on_runway >= 0 && cargo_on_runway <= MAX_RUNWAY_CAPACITY);
  assert(emergency_on_runway >= 0 && emergency_on_runway <= MAX_RUNWAY_CAPACITY);

  pthread_exit(NULL);
}

/* Main code for cargo aircraft threads.
 * You do not need to change anything here, but you can add
 * debug statements to help you during development/debugging.
 */
void* cargo_aircraft(void *ai_ptr) 
{
  aircraft_info *ai = (aircraft_info*)ai_ptr;
  
  /* Record arrival time for fuel tracking */
  ai->arrival_timestamp = time(NULL);

  /* Request runway access */
  cargo_enter(ai);

  printf("Cargo aircraft %d (fuel: %ds) is now on the runway (direction: %s)\n", 
         ai->aircraft_id, ai->fuel_reserve,
         current_direction == NORTH ? "NORTH" : "SOUTH");

  if (!(aircraft_on_runway <= MAX_RUNWAY_CAPACITY && aircraft_on_runway >= 0)) {
    printf("ASSERT FAILURE: aircraft_on_runway=%d (should be 0-%d)\n", aircraft_on_runway, 
            MAX_RUNWAY_CAPACITY);
    printf("Runway state: commercial=%d, cargo=%d, emergency=%d, direction=%s\n", 
           commercial_on_runway, cargo_on_runway, emergency_on_runway,
           current_direction == NORTH ? "NORTH" : "SOUTH");
  }
  assert(aircraft_on_runway <= MAX_RUNWAY_CAPACITY && aircraft_on_runway >= 0);
  assert(commercial_on_runway >= 0 && commercial_on_runway <= MAX_RUNWAY_CAPACITY);
  assert(cargo_on_runway >= 0 && cargo_on_runway <= MAX_RUNWAY_CAPACITY);
  assert(emergency_on_runway >= 0 && emergency_on_runway <= MAX_RUNWAY_CAPACITY);
  assert(commercial_on_runway == 0 ); 

  printf("Cargo aircraft %d begins runway operations for %d seconds\n", 
         ai->aircraft_id, ai->runway_time);
  use_runway(ai->runway_time);
  printf("Cargo aircraft %d completes runway operations and prepares to depart\n", 
         ai->aircraft_id);

  /* Leave runway */
  cargo_leave();        

  printf("Cargo aircraft %d has cleared the runway\n", ai->aircraft_id);

  if (!(aircraft_on_runway <= MAX_RUNWAY_CAPACITY && aircraft_on_runway >= 0)) {
    printf("ASSERT FAILURE: aircraft_on_runway=%d (should be 0-%d)\n", 
           aircraft_on_runway, MAX_RUNWAY_CAPACITY);
    printf("Runway state: commercial=%d, cargo=%d, emergency=%d, direction=%s\n", 
           commercial_on_runway, cargo_on_runway, emergency_on_runway,
           current_direction == NORTH ? "NORTH" : "SOUTH");
  }
  assert(aircraft_on_runway <= MAX_RUNWAY_CAPACITY && aircraft_on_runway >= 0);
  assert(commercial_on_runway >= 0 && commercial_on_runway <= MAX_RUNWAY_CAPACITY);
  assert(cargo_on_runway >= 0 && cargo_on_runway <= MAX_RUNWAY_CAPACITY);
  assert(emergency_on_runway >= 0 && emergency_on_runway <= MAX_RUNWAY_CAPACITY);

  pthread_exit(NULL);
}

/* Main code for emergency aircraft threads.
 * You do not need to change anything here, but you can add
 * debug statements to help you during development/debugging.
 */
void* emergency_aircraft(void *ai_ptr) 
{
  aircraft_info *ai = (aircraft_info*)ai_ptr;
  
  /* Record arrival time for fuel and emergency timeout tracking */
  ai->arrival_timestamp = time(NULL);

  /* Request runway access */
  emergency_enter(ai);

  printf("EMERGENCY aircraft %d (fuel: %ds) is now on the runway (direction: %s)\n", 
         ai->aircraft_id, ai->fuel_reserve,
         current_direction == NORTH ? "NORTH" : "SOUTH");

  if (!(aircraft_on_runway <= MAX_RUNWAY_CAPACITY && aircraft_on_runway >= 0)) {
    printf("ASSERT FAILURE: aircraft_on_runway=%d (should be 0-%d)\n", aircraft_on_runway, 
            MAX_RUNWAY_CAPACITY);
    printf("Runway state: commercial=%d, cargo=%d, emergency=%d, direction=%s\n", 
           commercial_on_runway, cargo_on_runway, emergency_on_runway,
           current_direction == NORTH ? "NORTH" : "SOUTH");
  }
  assert(aircraft_on_runway <= MAX_RUNWAY_CAPACITY && aircraft_on_runway >= 0);
  assert(commercial_on_runway >= 0 && commercial_on_runway <= MAX_RUNWAY_CAPACITY);
  assert(cargo_on_runway >= 0 && cargo_on_runway <= MAX_RUNWAY_CAPACITY);
  assert(emergency_on_runway >= 0 && emergency_on_runway <= MAX_RUNWAY_CAPACITY);

  printf("EMERGENCY aircraft %d begins runway operations for %d seconds\n", 
         ai->aircraft_id, ai->runway_time);
  use_runway(ai->runway_time);
  printf("EMERGENCY aircraft %d completes runway operations and prepares to depart\n", 
         ai->aircraft_id);

  /* Leave runway */
  emergency_leave();        

  printf("EMERGENCY aircraft %d has cleared the runway\n", ai->aircraft_id);

  if (!(aircraft_on_runway <= MAX_RUNWAY_CAPACITY && aircraft_on_runway >= 0)) {
    printf("ASSERT FAILURE: aircraft_on_runway=%d (should be 0-%d)\n", 
           aircraft_on_runway, MAX_RUNWAY_CAPACITY);
    printf("Runway state: commercial=%d, cargo=%d, emergency=%d, direction=%s\n", 
           commercial_on_runway, cargo_on_runway, emergency_on_runway,
           current_direction == NORTH ? "NORTH" : "SOUTH");
  }
  assert(aircraft_on_runway <= MAX_RUNWAY_CAPACITY && aircraft_on_runway >= 0);
  assert(commercial_on_runway >= 0 && commercial_on_runway <= MAX_RUNWAY_CAPACITY);
  assert(cargo_on_runway >= 0 && cargo_on_runway <= MAX_RUNWAY_CAPACITY);
  assert(emergency_on_runway >= 0 && emergency_on_runway <= MAX_RUNWAY_CAPACITY);

  pthread_exit(NULL);
}

/* Main function sets up simulation and prints report
 * at the end.
 * GUID: 355F4066-DA3E-4F74-9656-EF8097FBC985
 */
int main(int nargs, char **args) 
{
  int i;
  int result;
  int num_aircraft;
  void *status;
  pthread_t controller_tid;
  pthread_t aircraft_tid[MAX_AIRCRAFT];
  aircraft_info ai[MAX_AIRCRAFT];

  if (nargs != 2) 
  {
    printf("Usage: runway <name of inputfile>\n");
    return EINVAL;
  }

  num_aircraft = initialize(ai, args[1]);
  if (num_aircraft > MAX_AIRCRAFT || num_aircraft <= 0) 
  {
    printf("Error:  Bad number of aircraft threads. "
           "Maybe there was a problem with your input file?\n");
    return 1;
  }

  printf("Starting runway simulation with %d aircraft ...\n", num_aircraft);

  result = pthread_create(&controller_tid, NULL, controller_thread, NULL);

  if (result) 
  {
    printf("runway:  pthread_create failed for controller: %s\n", strerror(result));
    exit(1);
  }

  for (i=0; i < num_aircraft; i++) 
  {
    ai[i].aircraft_id = i;
    sleep(ai[i].arrival_time);
                
    if (ai[i].aircraft_type == COMMERCIAL)
    {
      result = pthread_create(&aircraft_tid[i], NULL, commercial_aircraft, 
                             (void *)&ai[i]);
    }
    else if (ai[i].aircraft_type == CARGO)
    {
      result = pthread_create(&aircraft_tid[i], NULL, cargo_aircraft, 
                             (void *)&ai[i]);
    }
    else 
    {
      result = pthread_create(&aircraft_tid[i], NULL, emergency_aircraft, 
                             (void *)&ai[i]);
    }

    if (result) 
    {
      printf("runway: pthread_create failed for aircraft %d: %s\n", 
            i, strerror(result));
      exit(1);
    }
  }

  /* wait for all aircraft threads to finish */
  for (i = 0; i < num_aircraft; i++) 
  {
    pthread_join(aircraft_tid[i], &status);
  }

  /* tell the controller to finish. */
  pthread_cancel(controller_tid);
  pthread_join(controller_tid, &status);

  printf("Runway simulation done.\n");

  return 0;
}