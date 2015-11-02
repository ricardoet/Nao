/*
 * CopyrightSensor (c) 2012-2014 Aldebaran Robotics. All rightSensors reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */
#include <iostream>
#include <alproxies/alsonarproxy.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/alrobotpostureproxy.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/allandmarkdetectionproxy.h>
#include <alproxies/altexttospeechproxy.h>
#include <alproxies/alnavigationproxy.h>
#include <alproxies/alvideodeviceproxy.h>
#include <alvision/alimage.h>
#include <alcommon/albroker.h>
#include <unistd.h>
#include <math.h>
#include <cmath>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <numeric>
#include <algorithm>
#include <sys/time.h>

#define _USE_MATH_DEFINES //PI
#define distToWall 0.30 //Distance to the wall to stop at
#define distToTurn 0.35 //Distance to the wall to stop at
#define distToWallAtTurning .5
#define ofstRegDiffSensors .15
#define clearPathDistance 0.5
#define spin90rightSensor ((-M_PI/2)+0.26) // Turn rightSensor PI/2 Radians +- error
#define spin90leftSensor ((M_PI/2)-0.17)// Turn leftSensor PI/2 Radians +- error
#define spin180leftSensor (M_PI-0.52) //30 grados
#define ofstToTurn .1 //Offset to turn
#define acceptedDiffSensors .2
#define acceptedDiffSensorsAtSpin .15
#define ofstRegDiffSensors .15

using namespace std;
using namespace cv; // Opencv namespace
using namespace AL; // Aldebaran namespace

struct timeval start, end;

//Variables globales

//Variable que contiene los posibles estados del robotNao
String states[16]={"Advancing",
                  "Finished",
                  "Crashed",
                  "Going Backwards",
                  "Turning",
                  "TurnRight",
                  "TurnLeft",
                  "Go Back",
                  "Aligning",
                  "RegularAligning",
                  "SpinBackLeft",
                  "SpinBackRight",
                  "Aligned",
                  "SpecialAligning",
                  "MoveSideWaysRight",
                  "MoveSideWaysLeft"};

//Variables que hacen referencia al estado actual y al estado pasado del robotNao
int currentState = 0;
int oldState = 0;

//Variables utilizadas para guardar la distancia regresada por los sensores ultrasonicos
float leftSensor = 0.0;
float rightSensor = 0.0;

//Variable utilizada para saber si es la primera vez que entra al estado
bool newState = true;

//Variablepara limitar el numero de Spins que puede realizar en un aligment
int maxSpins =3;

// Timer to turn
float timer = 0;

/*
*  updadateUS
*
*  Metodo encargado de actualizar las distancias proporcionadas por los sensores
*  ultrasonicos
*
*  @param
*/
void updateUS(AL::ALMemoryProxy memory){

    //leftSensor = memory.getData("Device/SubDeviceList/US/Left/Sensor/Value");
    //rightSensor = memory.getData("Device/SubDeviceList/US/Right/Sensor/Value");
    vector<float>leftSensorLectures;
    vector<float>rightSensorLectures;

    //Obtiene 5 nuevas lecturas y las almacena en un vector
    for (int i=0; i < 5; i++){
        leftSensorLectures.push_back(memory.getData("Device/SubDeviceList/US/Left/Sensor/Value"));
                          rightSensorLectures.push_back(memory.getData("Device/SubDeviceList/US/Right/Sensor/Value"));
      sleep(0.1);
    }
    sort(leftSensorLectures.begin(),leftSensorLectures.end());
    sort(rightSensorLectures.begin(),rightSensorLectures.end());
    leftSensor = leftSensorLectures.at(2);
    rightSensor = rightSensorLectures.at(2);
    leftSensorLectures.clear();
    rightSensorLectures.clear();


}

/*
 * ClockManager
 *
 * Actualiza el timer
 *
 */
void clockManager(int option)
{
    gettimeofday(&end, NULL);
    float elapsedTime = (end.tv_sec - start.tv_sec);
    elapsedTime += (end.tv_usec - start.tv_usec) / 1000000.0;
    /*
     * 1: add
     * 2: substract
     * 3: reset
     */
    switch (option) {
    case 1:
        timer = timer + (elapsedTime);
        break;
    case 2:
        timer = timer - (elapsedTime);
        break;
    case 3:
        timer = 0;
        break;
    default:
        break;
    }
}

/*
*	advancing
*
*	description
*
*	@param
*/
void advancing(AL::ALMotionProxy movement, AL::ALMemoryProxy memory, AL::ALTextToSpeechProxy say){
    //Ejecucion de codigo
    if (!movement.moveIsActive()){
        //say.post.say("Here I come");
        movement.move(0.1,0,0);
    }
    //Validaciones para cambio de estaado


    //Valores de sensores utilizados en validaciones
    float middleTactil = memory.getData("MiddleTactilTouched");
    float bumperRight = memory.getData("RightBumperPressed");
    float bumperLeft = memory.getData("LeftBumperPressed");
    float diffSensor = fabs(rightSensor-leftSensor);
    float minSensor = min(rightSensor, leftSensor);
    float percentage = diffSensor/minSensor;

    //Cambio a Finished
    if (middleTactil>0){
        oldState = currentState;
        currentState = 1;
        newState = true;
    }
    //Cambio a Crashed
    else if (bumperLeft>10 ||bumperRight>10 ){
        oldState = currentState;
        currentState = 2;
        newState = true;
    }
    //Cambio a Going Backwards
    else if (leftSensor <= distToWall || rightSensor <= distToWall){
        oldState = currentState;
        currentState = 3;
        newState = true;
    }
    //Cambio a Turning
    else if (leftSensor <= distToTurn && rightSensor <= distToTurn){
        oldState = currentState;
        currentState = 4;
        newState = true;
    }
    //Cambio de Estado a Turn Right **VALIDACION ESPECIAL**
    else if ( timer > 7.0 && leftSensor > clearPathDistance && rightSensor > clearPathDistance) {
            timer = 0.0;
            oldState = currentState;
            currentState = 5;
    }
    //Cambio a Aligning
    else if (percentage > acceptedDiffSensors && (leftSensor <clearPathDistance && rightSensor < clearPathDistance )) {
        oldState = currentState;
        currentState = 8;
        newState = true;
    }

}

/*
*	finished
*
*	description
*
*	@param
*/
void finished(AL::ALMotionProxy movement, AL::ALTextToSpeechProxy say){
    //Ejecucion de codigo
    say.post.say("I am awesome");
    movement.stopMove();
    exit(0);
}

/*
*	crashed
*
*	description
*
*	@param
*/
void crashed(AL::ALTextToSpeechProxy say){
    //Ejecucion de codigo
    say.post.say("Ouch");

    //Validaciones para cambio de estaado
    oldState = currentState;
    currentState = 3;
    newState = true;
}

/*
*	goingBackwards
*
*	description
*
*	@param
*/
void goingBackwards(AL::ALMotionProxy movement,AL::ALMemoryProxy memory, AL::ALTextToSpeechProxy say){
    //Ejecucion de codigo
    if (oldState == 2){
        float bumperRight = memory.getData("RightBumperPressed");
        float bumperLeft = memory.getData("LeftBumperPressed");
        movement.stopMove();
        if (bumperRight > 0) {

            movement.moveTo(-0.25,0,M_PI/16);
            movement.moveTo(0,0.1,0);
        }
        else{
            movement.moveTo(-0.25,0,-M_PI/16);
            movement.moveTo(0,-0.1,0);
        }
    }
    else if(oldState == 0 ){
        //say.post.say("Too close");
        movement.stopMove();
        if (leftSensor <= distToWall && rightSensor <= distToWall){
            movement.moveTo(-0.1,0,0);
        }
        else if (leftSensor<rightSensor){
            movement.moveTo(-0.1,0,-M_PI/16);
        }else{
            movement.moveTo(-0.1,0,M_PI/16);
        }


    }
    updateUS(memory);
    //Validaciones para cambio de estaado

    //Cambio a Aligning
    //if (leftSensor >= distToWall && rightSensor >= distToWall){
        oldState = currentState;
        currentState = 8;
        newState = true;
    //}
}

/*
*	turning
*
*	description
*
*	@param
*/
void turning(AL::ALMotionProxy movement){
    //Ejecucion de codigo
    if (movement.moveIsActive()){
        movement.stopMove();
    }

    //Valores de sensores utilizados en validaciones
    float diffSensor = fabs(rightSensor-leftSensor);
    float minSensor = min(rightSensor, leftSensor);
    float percentage = diffSensor/minSensor;

    //Validaciones para cambio de estaado

    //Cambio a Aligning
    if (percentage > acceptedDiffSensors ) {
        oldState = currentState;
        currentState = 8;
        newState = true;
    }
    //Cambio a TurnRight
    else{
        oldState = currentState;
        currentState = 5;
        newState = true;
    }
}

/*
*	turnRight
*
*	description
*
*	@param
*/
void turnRight(AL::ALMotionProxy movement, AL::ALMemoryProxy memory, AL::ALTextToSpeechProxy say){
    //Ejecucion de codigo
    // Giro a la derecha
    say.post.say("Let's check right");
    movement.moveTo(0,0,spin90rightSensor);
    updateUS(memory);


    //Validaciones para cambio de estaado

    //VALIDACION ESPECIAL 7 SEG sigue su camino
    if (oldState == 0 && leftSensor < distToWallAtTurning && rightSensor < distToWallAtTurning) {
        movement.moveTo(0,0,spin90leftSensor);
        updateUS(memory);
        oldState = currentState;
        currentState = 0;
        newState = true;
    }
    //Cambio a Turn Left
    else if (leftSensor < distToWallAtTurning && rightSensor < distToWallAtTurning) {
        oldState = currentState;
        currentState = 6;
        newState = true;
    }
    //Cambio a Advancing
    else{
        oldState = currentState;
        currentState = 0;
        newState = true;
    }
}

/*
*	turnLeft
*
*	description
*
*	@param
*/
void turnLeft(AL::ALMotionProxy movement,AL::ALMemoryProxy memory, AL::ALTextToSpeechProxy say){
    //Ejecucion de codigo

    // Giro a la izquierda
    say.post.say("Let's check left");
    movement.moveTo(0,0,spin180leftSensor);
    updateUS(memory);

    //Validaciones para cambio de estaado

    //Cambio a Go Back
    if (leftSensor < distToWallAtTurning && rightSensor < distToWallAtTurning) {
        oldState = currentState;
        currentState = 7;
        newState = true;
    }
    //Cambio a Advancing
    else{
        oldState = currentState;
        currentState = 0;
        newState = true;
    }
}

/*
*	goBack
*
*	description
*
*	@param
*/
void goBack(AL::ALMotionProxy movement, AL::ALTextToSpeechProxy say){
    //Ejecucion de codigo

    // Giro a la derecha
    say.post.say("Dead alley. Lets get out of here.");
    movement.moveTo(0,0,spin90leftSensor);

    //Validaciones para cambio de estaado

    //Cambio a Advancing
    oldState = currentState;
    currentState = 0;
    newState = true;

}

/*
*	aligning
*
*	description
*
*	@param
*/
void aligning(AL::ALMotionProxy movement){
    //Ejecucion de codigo
    if (movement.moveIsActive()){
        movement.stopMove();
    }

    //Valores de sensores utilizados en validaciones
    float diffSensor = fabs(rightSensor-leftSensor);
    float minSensor = min(rightSensor, leftSensor);
    float percentage = diffSensor/minSensor;


    //Validaciones para cambio de estaado

    //Cambio a Special Aligning
    if (percentage > 4 ) {
        oldState = currentState;
        currentState = 13;
        newState = true;
    }
    //Cambio a Regular Aligning
    else{

        oldState = currentState;
        currentState = 9;
        newState = true;
    }
}

/*
*	regularAligning
*
*	description
*
*	@param
*/
void regularAligning(AL::ALMotionProxy movement, AL::ALTextToSpeechProxy say){

    //Validaciones para cambio de estaado

    //Cambio a SpinBackLeft
    if (rightSensor < leftSensor ) {
        oldState = currentState;
        currentState = 10;
        newState = true;
    }
    //Cambio a SpinBackRight
    else{
        oldState = currentState;
        currentState = 11;
        newState = true;
    }
}

/*
*	spinBackLeft
*
*	description
*
*	@param
*/
void spinBackLeft(AL::ALMotionProxy movement){
    //Ejecucion de codigo
    if (!movement.moveIsActive()){
        movement.move(-0.05,0,M_PI/16);
        maxSpins++;

    }

    //Validaciones para cambio de estaado

    //Valores de sensores utilizados en validaciones
    float diffSensor = fabs(rightSensor-leftSensor);
    float minSensor = min(rightSensor, leftSensor);
    float percentage = diffSensor/minSensor;

    //Validaciones para cambio de estaado

    //Cambio a Aligned
    if (percentage <= acceptedDiffSensorsAtSpin || maxSpins>=3) {
        oldState = currentState;
        currentState = 12;
        newState = true;
    }
}

/*
*	spinBackRight
*
*	description
*
*	@param
*/
void spinBackRight(AL::ALMotionProxy movement){
    //Ejecucion de codigo
    if (!movement.moveIsActive()){
        movement.move(-0.05,0,-M_PI/16);
        maxSpins++;
    }
    //Validaciones para cambio de estaado

    //Valores de sensores utilizados en validaciones
    float diffSensor = fabs(rightSensor-leftSensor);
    float minSensor = min(rightSensor, leftSensor);
    float percentage = diffSensor/minSensor;

    //Validaciones para cambio de estaado

    //Cambio a Aligned
    if (percentage <= acceptedDiffSensorsAtSpin || maxSpins>=3 ) {
        oldState = currentState;
        currentState = 12;
        newState = true;
        maxSpins=0;
    }
}

/*
*	aligned
*
*	description
*
*	@param
*/
void aligned(AL::ALMotionProxy movement){
    //Ejecucion de codigo
    if (movement.moveIsActive()){
        movement.stopMove();
    }

    //Validaciones para cambio de estaado
    oldState = currentState;
    currentState = 0;
    newState = true;
}

/*
*	specialAligning
*
*	description
*
*	@param
*/

void specialAligning(AL::ALTextToSpeechProxy say){
    //Ejecucion de codigo
    say.post.say("I dont edges");

    //Valores de sensores utilizados en validaciones
    float maxSensor = max(rightSensor, leftSensor);

    //Validaciones para cambio de estaado

    //Cambio a MoveSideWaysRight
    if (maxSensor == rightSensor ) {
        oldState = currentState;
        currentState = 14;
        newState = true;
    }
    //Cambio a MoveSideWaysLeft
    else{
        oldState = currentState;
        currentState = 15;
        newState = true;
    }
}
/*
*	moveSideWaysRight
*
*	description
*
*	@param
*/
void moveSideWaysRight(AL::ALMotionProxy movement){
    //Ejecucion de codigo

    if (!movement.moveIsActive()){
        movement.moveTo(0,-0.2,0);
        movement.move(0,0,-M_PI/20);
    }

    //Valores de sensores utilizados en validaciones
    float diffSensor = fabs(rightSensor-leftSensor);
    float minSensor = min(rightSensor, leftSensor);
    float percentage = diffSensor/minSensor;

    //Validaciones para cambio de estaado
    //Cambio a Regular Aligning
    if (percentage >acceptedDiffSensors && percentage <acceptedDiffSensors+ofstRegDiffSensors ) {
        oldState = currentState;
        currentState = 9;
        newState = true;
        movement.stopMove();
    }
}

/*
*	moveSideWaysLeft
*
*	description
*
*	@param
*/
void moveSideWaysLeft(AL::ALMotionProxy movement){
    //Ejecucion de codigo

    if (!movement.moveIsActive()){
        movement.moveTo(0,0.2,0);
        movement.move(0,0,M_PI/20);
    }

    //Valores de sensores utilizados en validaciones
    float diffSensor = fabs(rightSensor-leftSensor);
    float minSensor = min(rightSensor, leftSensor);
    float percentage = diffSensor/minSensor;

    //Validaciones para cambio de estaado
    //Cambio a Regular Aligning
    if (percentage >acceptedDiffSensors && percentage <acceptedDiffSensors+ofstRegDiffSensors ) {
        oldState = currentState;
        currentState = 9;
        newState = true;
        movement.stopMove();
    }
}

/*
*  main
*
*  description
*
*  @param
*/
int main(int argc, char *argv[])
{
    //Obtencion de parametros de entrda
    String robotIP =argv[1];
    int port =9559;

    //Inicializacion de modulos
    AL::ALSonarProxy sonar(robotIP,port);
    AL::ALRobotPostureProxy posture(robotIP,port);
    AL::ALMotionProxy movement(robotIP,port);
    AL::ALLandMarkDetectionProxy naoMark(robotIP,port);
    AL::ALTextToSpeechProxy say(robotIP, port);
    AL::ALMemoryProxy memory(robotIP, port);

    //Suscripcion a modulos
    sonar.subscribe("ALSonar");
    int period = 500;
    naoMark.subscribe("Test_Mark", period, 0.0);


    // Inicializar postura
    bool stand;
    std::string  actualPosture;
    stand = posture.goToPosture("Stand",1);

    // Variables utilizadas en la deteccion de marcas
    Mat src, src_gray;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    //Frase inicial
    say.post.say("Winter is coming.");

    //Evaluacion de estados
    while(1)
    {
        // empieza timer
        gettimeofday(&start, NULL);

        //Impresion de estado actual en pantalla
        if (newState){
            cout<<"OldState:"<< states[oldState]<<"-->"<<"CurrentState: "<< states[currentState]<<endl;
            cout<<"LeftSensor:"<< leftSensor<<"|"<<"RightSensor: "<< rightSensor<<endl;
            newState = false;
        }

        cout<<"timer:"<<timer<<endl;

        //Actualizacion de valores de sensores ultrasonicos
        updateUS(memory);
        //Ingresa al estado correspondiente
        switch(currentState){
            case 0:
                advancing(movement, memory, say);
                clockManager(1);
                break;
            case 1:
                finished(movement,say);
                break;
            case 2:
                crashed(say);
                break;
            case 3:
                goingBackwards(movement,memory,say);
                clockManager(2);
                break;
            case 4:
                turning(movement);
                clockManager(3);
                break;
            case 5:
                turnRight(movement,memory, say);
                break;
            case 6:
                turnLeft(movement,memory, say);
                break;
            case 7:
                goBack(movement,say);
                break;
            case 8:
                aligning(movement);
                break;
            case 9:
                regularAligning(movement,say);
                clockManager(2);
                break;
            case 10:
                spinBackLeft(movement);
                clockManager(2);
                break;
            case 11:
                spinBackRight(movement);
                clockManager(2);
                break;
            case 12:
                aligned(movement);
                break;
            case 13:
                specialAligning(say);
                break;
            case 14:
                moveSideWaysRight(movement);
                clockManager(2);
                break;
            case 15:
                moveSideWaysLeft(movement);
                clockManager(2);
                break;
        }

        if (posture.getPostureFamily() != "Standing") {
            posture.goToPosture("Stand",1);
        }

    }

    return 0;
}
