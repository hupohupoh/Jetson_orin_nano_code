#include "led.h"
#include "delay.h"
#include "sys.h"
#include "exit.h" 
#include "LobotSerialServo.h"
#include "bool.h"
#include "pwm.h"
#include "action_new.h"

#define ID1 1
#define ID2 2
#define ID3 3
#define ID4 4
#define ID5 5
#define ID6 6
#define ID7 7
#define ID8 8
#define ID9 9
#define ID10 10


//void action_go_line(void)
//{
//	// #1 P497 #2 P498 #3 P447 #4 P459 #5 P500 #6 P495 #7 P537 #8 P499 #9 P498 #10 P541
//	LobotSerialServoMove(1, 497, 200);
//	LobotSerialServoMove(2, 498, 200);
//	LobotSerialServoMove(3, 447, 200);
//	LobotSerialServoMove(4, 459, 200);
//	LobotSerialServoMove(5, 500, 200);
//	LobotSerialServoMove(6, 495, 200);
//	LobotSerialServoMove(7, 537, 200);
//	LobotSerialServoMove(8, 499, 200);
//	LobotSerialServoMove(9, 498, 200);
//	LobotSerialServoMove(10, 541, 200);
//	delay_ms(205);

//    // #1 P503 #2 P500 #3 P448 #4 P463 #5 P498 #6 P494 #7 P536 #8 P447 #9 P389 #10 P545
//	LobotSerialServoMove(1, 503, 200);
//	LobotSerialServoMove(2, 500, 200);
//	LobotSerialServoMove(3, 448, 200);
//	LobotSerialServoMove(4, 463, 200);
//	LobotSerialServoMove(5, 498, 200);
//	LobotSerialServoMove(6, 494, 200);
//	LobotSerialServoMove(7, 536, 200);
//	LobotSerialServoMove(8, 447, 200);
//	LobotSerialServoMove(9, 389, 200);
//	LobotSerialServoMove(10, 545, 200);
//	delay_ms(205);

//	// #1 P503 #2 P502 #3 P590 #4 P463 #5 P486 #6 P493 #7 P420 #8 P451 #9 P441 #10 P546
//	LobotSerialServoMove(1, 503, 200);
//	LobotSerialServoMove(2, 502, 200);
//	LobotSerialServoMove(3, 590, 200);
//	LobotSerialServoMove(4, 463, 200);
//	LobotSerialServoMove(5, 486, 200);
//	LobotSerialServoMove(6, 493, 200);
//	LobotSerialServoMove(7, 420, 200);
//	LobotSerialServoMove(8, 451, 200);
//	LobotSerialServoMove(9, 441, 200);
//	LobotSerialServoMove(10, 546, 200);
//	delay_ms(205);

//	// #1 P518 #2 P503 #3 P540 #4 P401 #5 P511 #6 P492 #7 P450 #8 P507 #9 P512 #10 P595
//	LobotSerialServoMove(1, 518, 200);
//	LobotSerialServoMove(2, 503, 200);
//	LobotSerialServoMove(3, 540, 200);
//	LobotSerialServoMove(4, 401, 200);
//	LobotSerialServoMove(5, 511, 200);
//	LobotSerialServoMove(6, 492, 200);
//	LobotSerialServoMove(7, 450, 200);
//	LobotSerialServoMove(8, 507, 200);
//	LobotSerialServoMove(9, 512, 200);
//	LobotSerialServoMove(10, 595, 200);
//	delay_ms(205);

//	// #1 P511 #2 P520 #3 P546 #4 P421 #5 P533 #6 P443 #7 P512 #8 P553 #9 P520 #10 P601
//	LobotSerialServoMove(1, 511, 200);
//	LobotSerialServoMove(2, 520, 200);
//	LobotSerialServoMove(3, 546, 200);
//	LobotSerialServoMove(4, 421, 200);
//	LobotSerialServoMove(5, 533, 200);
//	LobotSerialServoMove(6, 443, 200);
//	LobotSerialServoMove(7, 512, 200);
//	LobotSerialServoMove(8, 553, 200);
//	LobotSerialServoMove(9, 520, 200);
//	LobotSerialServoMove(10, 601, 200);
//	delay_ms(205);

//	// #1 P511 #2 P517 #3 P543 #4 P419 #5 P529 #6 P449 #7 P513 #8 P651 #9 P557 #10 P601
//	LobotSerialServoMove(1, 511, 200);
//	LobotSerialServoMove(2, 517, 200);
//	LobotSerialServoMove(3, 543, 200);
//	LobotSerialServoMove(4, 419, 200);
//	LobotSerialServoMove(5, 529, 200);
//	LobotSerialServoMove(6, 449, 200);
//	LobotSerialServoMove(7, 513, 200);
//	LobotSerialServoMove(8, 651, 200);
//	LobotSerialServoMove(9, 557, 200);
//	LobotSerialServoMove(10, 601, 200);
//	delay_ms(205);

//	// #1 P494 #2 P500 #3 P543 #4 P595 #5 P589 #6 P464 #7 P513 #8 P521 #9 P555 #10 P546
//	LobotSerialServoMove(1, 494, 200);
//	LobotSerialServoMove(2, 500, 200);
//	LobotSerialServoMove(3, 543, 200);
//	LobotSerialServoMove(4, 595, 200);
//	LobotSerialServoMove(5, 589, 200);
//	LobotSerialServoMove(6, 464, 200);
//	LobotSerialServoMove(7, 513, 200);
//	LobotSerialServoMove(8, 521, 200);
//	LobotSerialServoMove(9, 555, 200);
//	LobotSerialServoMove(10, 546, 200);
//	delay_ms(205);
//}

void Guizhong_small(void)
{
	servo_angle1(180);
	servo_angle2(90);
	servo_angle3(90);
}

void Taizuoshou(void)
{
	servo_angle1(90);
	delay_ms(3000);
	servo_angle1(180);
}
void Taiyoushou (void)
{
	servo_angle2(180);
	delay_ms(3000);
	servo_angle2(90);
}
void Taishuangshou(void)
{
	servo_angle1(90);
	servo_angle2(180);
	delay_ms(3000);
	servo_angle1(180);
	servo_angle2(90);
}
void yaotou(void)
{
	servo_angle3(180);
	delay_ms(1000);
	servo_angle3(0);
	delay_ms(1000);
	servo_angle3(180);
	delay_ms(1000);
	servo_angle3(90);
}	

void new_guizhong(void)
{
// #1 P498 #2 P508 #3 P490 #4 P536 #5 P578 #6 P506 #7 P530 #8 P503 #9 P504 #10 P515
LobotSerialServoMove(1, 498, 300);
LobotSerialServoMove(2, 508, 300);
LobotSerialServoMove(3, 490, 300);
LobotSerialServoMove(4, 536, 300);
LobotSerialServoMove(5, 578, 300);
LobotSerialServoMove(6, 506, 300);
LobotSerialServoMove(7, 530, 300);
LobotSerialServoMove(8, 503, 300);
LobotSerialServoMove(9, 504, 300);
LobotSerialServoMove(10, 515, 300);
delay_ms(305);

}

//void new_zoulu(void)
//{
//// #1 P500 #2 P504 #3 P499 #4 P503 #5 P500 #6 P506 #7 P501 #8 P576 #9 P582 #10 P501
//LobotSerialServoMove(1, 525, 200);
//LobotSerialServoMove(2, 545, 200);
//LobotSerialServoMove(3, 499, 200);
//LobotSerialServoMove(4, 503, 200);
//LobotSerialServoMove(5, 500, 200);
//LobotSerialServoMove(6, 506, 200);
//LobotSerialServoMove(7, 501, 200);
//LobotSerialServoMove(8, 576, 200);
//LobotSerialServoMove(9, 575, 200);
//LobotSerialServoMove(10, 501, 200);
//delay_ms(205);

//// #1 P499 #2 P507 #3 P535 #4 P508 #5 P518 #6 P591 #7 P594 #8 P572 #9 P578 #10 P458
//LobotSerialServoMove(1, 525, 200);
//LobotSerialServoMove(2, 545, 200);
//LobotSerialServoMove(3, 515, 200);
//LobotSerialServoMove(4, 508, 200);
//LobotSerialServoMove(5, 511, 200);
//LobotSerialServoMove(6, 591, 200);
//LobotSerialServoMove(7, 594, 200);
//LobotSerialServoMove(8, 550, 200);
//LobotSerialServoMove(9, 560, 200);
//LobotSerialServoMove(10, 476, 200);
//delay_ms(205);

//// #1 P500 #2 P507 #3 P530 #4 P506 #5 P496 #6 P592 #7 P599 #8 P505 #9 P515 #10 P489
//LobotSerialServoMove(1, 525, 200);
//LobotSerialServoMove(2, 560, 200);
//LobotSerialServoMove(3, 530, 200);
//LobotSerialServoMove(4, 506, 200);
//LobotSerialServoMove(5, 496, 200);
//LobotSerialServoMove(6, 592, 200);
//LobotSerialServoMove(7, 599, 200);
//LobotSerialServoMove(8, 505, 200);
//LobotSerialServoMove(9, 515, 200);
//LobotSerialServoMove(10, 489, 200);
//delay_ms(205);

//// #1 P500 #2 P507 #3 P532 #4 P506 #5 P494 #6 P589 #7 P601 #8 P473 #9 P426 #10 P543
//LobotSerialServoMove(1, 525, 200);
//LobotSerialServoMove(2, 545, 200);
//LobotSerialServoMove(3, 532, 200);
//LobotSerialServoMove(4, 506, 200);
//LobotSerialServoMove(5, 494, 200);
//LobotSerialServoMove(6, 589, 200);
//LobotSerialServoMove(7, 601, 200);
//LobotSerialServoMove(8, 473, 200);
//LobotSerialServoMove(9, 426, 200);
//LobotSerialServoMove(10, 543, 200);
//delay_ms(205);

//// #1 P500 #2 P507 #3 P532 #4 P579 #5 P497 #6 P591 #7 P599 #8 P462 #9 P466 #10 P542
//LobotSerialServoMove(1, 525, 200);
//LobotSerialServoMove(2, 545, 200);
//LobotSerialServoMove(3, 532, 200);
//LobotSerialServoMove(4, 579, 200);
//LobotSerialServoMove(5, 497, 200);
//LobotSerialServoMove(6, 591, 200);
//LobotSerialServoMove(7, 599, 200);
//LobotSerialServoMove(8, 473, 200);
//LobotSerialServoMove(9, 466, 200);
//LobotSerialServoMove(10, 542, 200);
//delay_ms(205);

//// #1 P499 #2 P507 #3 P529 #4 P582 #5 P497 #6 P603 #7 P599 #8 P531 #9 P543 #10 P552
//LobotSerialServoMove(1, 525, 200);
//LobotSerialServoMove(2, 545, 200);
//LobotSerialServoMove(3, 529, 200);
//LobotSerialServoMove(4, 582, 200);
//LobotSerialServoMove(5, 497, 200);
//LobotSerialServoMove(6, 603, 200);
//LobotSerialServoMove(7, 599, 200);
//LobotSerialServoMove(8, 531, 200);
//LobotSerialServoMove(9, 543, 200);
//LobotSerialServoMove(10, 552, 200);
//delay_ms(205);

//}

void new_zoulu(void)
{
	
	//pianyou
//// #1 P502 #2 P502 #3 P502 #4 P500 #5 P505 #6 P505 #7 P538 #8 P444 #9 P432 #10 P540
//LobotSerialServoMove(1, 502, 200);
//LobotSerialServoMove(2, 502, 200);
//LobotSerialServoMove(3, 502, 200);
//LobotSerialServoMove(4, 500, 200);
//LobotSerialServoMove(5, 505, 200);
//LobotSerialServoMove(6, 505, 200);
//LobotSerialServoMove(7, 538, 200);
//LobotSerialServoMove(8, 444, 200);
//LobotSerialServoMove(9, 432, 200);
//LobotSerialServoMove(10, 540, 200);
//delay_ms(205);

//// #1 P541 #2 P534 #3 P502 #4 P558 #5 P501 #6 P503 #7 P538 #8 P442 #9 P431 #10 P535
//LobotSerialServoMove(1, 541, 200);
//LobotSerialServoMove(2, 534, 200);
//LobotSerialServoMove(3, 502, 200);
//LobotSerialServoMove(4, 558, 200);
//LobotSerialServoMove(5, 501, 200);
//LobotSerialServoMove(6, 503, 200);
//LobotSerialServoMove(7, 538, 200);
//LobotSerialServoMove(8, 442, 200);
//LobotSerialServoMove(9, 431, 200);
//LobotSerialServoMove(10, 535, 200);
//delay_ms(205);

//// #1 P533 #2 P534 #3 P502 #4 P552 #5 P533 #6 P502 #7 P516 #8 P508 #9 P509 #10 P536
//LobotSerialServoMove(1, 533, 200);
//LobotSerialServoMove(2, 534, 200);
//LobotSerialServoMove(3, 502, 200);
//LobotSerialServoMove(4, 552, 200);
//LobotSerialServoMove(5, 533, 200);
//LobotSerialServoMove(6, 502, 200);
//LobotSerialServoMove(7, 516, 200);
//LobotSerialServoMove(8, 508, 200);
//LobotSerialServoMove(9, 509, 200);
//LobotSerialServoMove(10, 536, 200);
//delay_ms(205);

//// #1 P531 #2 P533 #3 P502 #4 P553 #5 P532 #6 P503 #7 P548 #8 P567 #9 P585 #10 P537
//LobotSerialServoMove(1, 531, 300);
//LobotSerialServoMove(2, 533, 300);
//LobotSerialServoMove(3, 502, 300);
//LobotSerialServoMove(4, 553, 300);
//LobotSerialServoMove(5, 532, 300);
//LobotSerialServoMove(6, 503, 300);
//LobotSerialServoMove(7, 548, 300);
//LobotSerialServoMove(8, 567, 300);
//LobotSerialServoMove(9, 585, 300);
//LobotSerialServoMove(10, 537, 300);
//delay_ms(305);

//// #1 P530 #2 P533 #3 P513 #4 P501 #5 P531 #6 P502 #7 P560 #8 P564 #9 P578 #10 P537
//LobotSerialServoMove(1, 530, 200);
//LobotSerialServoMove(2, 533, 200);
//LobotSerialServoMove(3, 513, 200);
//LobotSerialServoMove(4, 501, 200);
//LobotSerialServoMove(5, 531, 200);
//LobotSerialServoMove(6, 502, 200);
//LobotSerialServoMove(7, 560, 200);
//LobotSerialServoMove(8, 564, 200);
//LobotSerialServoMove(9, 578, 200);
//LobotSerialServoMove(10, 537, 200);
//delay_ms(205);

//// #1 P498 #2 P514 #3 P511 #4 P549 #5 P597 #6 P476 #7 P535 #8 P500 #9 P508 #10 P508
//LobotSerialServoMove(1, 498, 200);
//LobotSerialServoMove(2, 514, 200);
//LobotSerialServoMove(3, 511, 200);
//LobotSerialServoMove(4, 549, 200);
//LobotSerialServoMove(5, 597, 200);
//LobotSerialServoMove(6, 476, 200);
//LobotSerialServoMove(7, 535, 200);
//LobotSerialServoMove(8, 500, 200);
//LobotSerialServoMove(9, 508, 200);
//LobotSerialServoMove(10, 508, 200);
//delay_ms(205);

//guizhong
//LobotSerialServoMove(1, 498, 300);
//LobotSerialServoMove(2, 508, 300);
//LobotSerialServoMove(3, 490, 300);
//LobotSerialServoMove(4, 536, 300);
//LobotSerialServoMove(5, 578, 300);
//LobotSerialServoMove(6, 506, 300);
//LobotSerialServoMove(7, 530, 300);
//LobotSerialServoMove(8, 503, 300);
//LobotSerialServoMove(9, 504, 300);
//LobotSerialServoMove(10, 515, 300);
//delay_ms(305);


//pianzou

// #1 P500 #2 P504 #3 P499 #4 P503 #5 P500 #6 P506 #7 P501 #8 P576 #9 P582 #10 P501
LobotSerialServoMove(1, 525, 200);
LobotSerialServoMove(2, 545, 200);
LobotSerialServoMove(3, 499, 200);
LobotSerialServoMove(4, 503, 200);
LobotSerialServoMove(5, 500, 200);
LobotSerialServoMove(6, 506, 200);
LobotSerialServoMove(7, 501, 200);
LobotSerialServoMove(8, 576, 200);
LobotSerialServoMove(9, 575, 200);
LobotSerialServoMove(10, 501, 200);
delay_ms(205);

// #1 P499 #2 P507 #3 P535 #4 P508 #5 P518 #6 P591 #7 P594 #8 P572 #9 P578 #10 P458
LobotSerialServoMove(1, 525, 200);
LobotSerialServoMove(2, 545, 200);
LobotSerialServoMove(3, 515, 200);
LobotSerialServoMove(4, 508, 200);
LobotSerialServoMove(5, 511, 200);
LobotSerialServoMove(6, 591, 200);
LobotSerialServoMove(7, 594, 200);
LobotSerialServoMove(8, 550, 200);
LobotSerialServoMove(9, 560, 200);
LobotSerialServoMove(10, 476, 200);
delay_ms(205);

// #1 P500 #2 P507 #3 P530 #4 P506 #5 P496 #6 P592 #7 P599 #8 P505 #9 P515 #10 P489
LobotSerialServoMove(1, 525, 200);
LobotSerialServoMove(2, 560, 200);
LobotSerialServoMove(3, 530, 200);
LobotSerialServoMove(4, 506, 200);
LobotSerialServoMove(5, 496, 200);
LobotSerialServoMove(6, 592, 200);
LobotSerialServoMove(7, 599, 200);
LobotSerialServoMove(8, 505, 200);
LobotSerialServoMove(9, 515, 200);
LobotSerialServoMove(10, 489, 200);
delay_ms(205);

// #1 P500 #2 P507 #3 P532 #4 P506 #5 P494 #6 P589 #7 P601 #8 P473 #9 P426 #10 P543
LobotSerialServoMove(1, 525, 200);
LobotSerialServoMove(2, 545, 200);
LobotSerialServoMove(3, 532, 200);
LobotSerialServoMove(4, 506, 200);
LobotSerialServoMove(5, 494, 200);
LobotSerialServoMove(6, 589, 200);
LobotSerialServoMove(7, 601, 200);
LobotSerialServoMove(8, 473, 200);
LobotSerialServoMove(9, 426, 200);
LobotSerialServoMove(10, 543, 200);
delay_ms(205);

// #1 P500 #2 P507 #3 P532 #4 P579 #5 P497 #6 P591 #7 P599 #8 P462 #9 P466 #10 P542
LobotSerialServoMove(1, 525, 200);
LobotSerialServoMove(2, 545, 200);
LobotSerialServoMove(3, 532, 200);
LobotSerialServoMove(4, 579, 200);
LobotSerialServoMove(5, 497, 200);
LobotSerialServoMove(6, 591, 200);
LobotSerialServoMove(7, 599, 200);
LobotSerialServoMove(8, 473, 200);
LobotSerialServoMove(9, 466, 200);
LobotSerialServoMove(10, 542, 200);
delay_ms(205);

// #1 P499 #2 P507 #3 P529 #4 P582 #5 P497 #6 P603 #7 P599 #8 P531 #9 P543 #10 P552
LobotSerialServoMove(1, 525, 200);
LobotSerialServoMove(2, 545, 200);
LobotSerialServoMove(3, 529, 200);
LobotSerialServoMove(4, 582, 200);
LobotSerialServoMove(5, 497, 200);
LobotSerialServoMove(6, 603, 200);
LobotSerialServoMove(7, 599, 200);
LobotSerialServoMove(8, 531, 200);
LobotSerialServoMove(9, 543, 200);
LobotSerialServoMove(10, 552, 200);
delay_ms(205);




//// #1 P492 #2 P502 #3 P498 #4 P498 #5 P466 #6 P520 #7 P565 #8 P563 #9 P585 #10 P576
//LobotSerialServoMove(1, 492, 300);
//LobotSerialServoMove(2, 502, 300);
//LobotSerialServoMove(3, 498, 300);
//LobotSerialServoMove(4, 498, 300);
//LobotSerialServoMove(5, 466, 300);
//LobotSerialServoMove(6, 520, 300);
//LobotSerialServoMove(7, 565, 300);
//LobotSerialServoMove(8, 563, 300);
//LobotSerialServoMove(9, 585, 300);
//LobotSerialServoMove(10, 576, 300);
//delay_ms(305);

//// #1 P492 #2 P501 #3 P609 #4 P499 #5 P413 #6 P518 #7 P567 #8 P564 #9 P582 #10 P576
//LobotSerialServoMove(1, 492, 300);
//LobotSerialServoMove(2, 501, 300);
//LobotSerialServoMove(3, 609, 300);
//LobotSerialServoMove(4, 499, 300);
//LobotSerialServoMove(5, 413, 300);
//LobotSerialServoMove(6, 518, 300);
//LobotSerialServoMove(7, 567, 300);
//LobotSerialServoMove(8, 564, 300);
//LobotSerialServoMove(9, 582, 300);
//LobotSerialServoMove(10, 576, 300);
//delay_ms(305);

//// #1 P493 #2 P502 #3 P605 #4 P494 #5 P396 #6 P522 #7 P607 #8 P494 #9 P522 #10 P579
//LobotSerialServoMove(1, 493, 300);
//LobotSerialServoMove(2, 502, 300);
//LobotSerialServoMove(3, 605, 300);
//LobotSerialServoMove(4, 494, 300);
//LobotSerialServoMove(5, 396, 300);
//LobotSerialServoMove(6, 522, 300);
//LobotSerialServoMove(7, 607, 300);
//LobotSerialServoMove(8, 494, 300);
//LobotSerialServoMove(9, 522, 300);
//LobotSerialServoMove(10, 579, 300);
//delay_ms(305);

//// #1 P495 #2 P504 #3 P604 #4 P494 #5 P396 #6 P523 #7 P605 #8 P438 #9 P447 #10 P577
//LobotSerialServoMove(1, 495, 300);
//LobotSerialServoMove(2, 504, 300);
//LobotSerialServoMove(3, 604, 300);
//LobotSerialServoMove(4, 494, 300);
//LobotSerialServoMove(5, 396, 300);
//LobotSerialServoMove(6, 523, 300);
//LobotSerialServoMove(7, 605, 300);
//LobotSerialServoMove(8, 438, 300);
//LobotSerialServoMove(9, 447, 300);
//LobotSerialServoMove(10, 577, 300);
//delay_ms(305);

//// #1 P493 #2 P504 #3 P605 #4 P602 #5 P376 #6 P523 #7 P521 #8 P438 #9 P448 #10 P579
//LobotSerialServoMove(1, 493, 300);
//LobotSerialServoMove(2, 504, 300);
//LobotSerialServoMove(3, 605, 300);
//LobotSerialServoMove(4, 602, 300);
//LobotSerialServoMove(5, 376, 300);
//LobotSerialServoMove(6, 523, 300);
//LobotSerialServoMove(7, 521, 300);
//LobotSerialServoMove(8, 438, 300);
//LobotSerialServoMove(9, 448, 300);
//LobotSerialServoMove(10, 579, 300);
//delay_ms(305);

//// #1 P496 #2 P501 #3 P498 #4 P495 #5 P465 #6 P522 #7 P560 #8 P499 #9 P498 #10 P571
//LobotSerialServoMove(1, 496, 500);
//LobotSerialServoMove(2, 501, 500);
//LobotSerialServoMove(3, 498, 500);
//LobotSerialServoMove(4, 495, 500);
//LobotSerialServoMove(5, 465, 500);
//LobotSerialServoMove(6, 522, 500);
//LobotSerialServoMove(7, 560, 500);
//LobotSerialServoMove(8, 499, 500);
//LobotSerialServoMove(9, 498, 500);
//LobotSerialServoMove(10, 571, 500);
//delay_ms(505);

}





void new_taiyoutui(void)
{
// #1 P499 #2 P501 #3 P505 #4 P500 #5 P499 #6 P502 #7 P539 #8 P444 #9 P427 #10 P551
LobotSerialServoMove(1, 499, 200);
LobotSerialServoMove(2, 501, 200);
LobotSerialServoMove(3, 505, 200);
LobotSerialServoMove(4, 500, 200);
LobotSerialServoMove(5, 499, 200);
LobotSerialServoMove(6, 502, 200);
LobotSerialServoMove(7, 539, 200);
LobotSerialServoMove(8, 444, 200);
LobotSerialServoMove(9, 427, 200);
LobotSerialServoMove(10, 551, 200);
delay_ms(205);

// #1 P504 #2 P501 #3 P507 #4 P680 #5 P505 #6 P673 #7 P542 #8 P435 #9 P428 #10 P560
LobotSerialServoMove(1, 504, 200);
LobotSerialServoMove(2, 501, 200);
LobotSerialServoMove(3, 507, 200);
LobotSerialServoMove(4, 680, 200);
LobotSerialServoMove(5, 505, 200);
LobotSerialServoMove(6, 673, 200);
LobotSerialServoMove(7, 542, 200);
LobotSerialServoMove(8, 435, 200);
LobotSerialServoMove(9, 428, 200);
LobotSerialServoMove(10, 560, 200);
delay_ms(205);

// #1 P504 #2 P501 #3 P507 #4 P680 #5 P505 #6 P674 #7 P542 #8 P435 #9 P428 #10 P560
LobotSerialServoMove(1, 504, 3000);
LobotSerialServoMove(2, 501, 3000);
LobotSerialServoMove(3, 507, 3000);
LobotSerialServoMove(4, 680, 3000);
LobotSerialServoMove(5, 505, 3000);
LobotSerialServoMove(6, 674, 3000);
LobotSerialServoMove(7, 542, 3000);
LobotSerialServoMove(8, 435, 3000);
LobotSerialServoMove(9, 428, 3000);
LobotSerialServoMove(10, 560, 3000);
delay_ms(3005);

// #1 P501 #2 P501 #3 P504 #4 P504 #5 P518 #6 P504 #7 P556 #8 P509 #9 P496 #10 P553
LobotSerialServoMove(1, 501, 200);
LobotSerialServoMove(2, 501, 200);
LobotSerialServoMove(3, 504, 200);
LobotSerialServoMove(4, 504, 200);
LobotSerialServoMove(5, 518, 200);
LobotSerialServoMove(6, 504, 200);
LobotSerialServoMove(7, 556, 200);
LobotSerialServoMove(8, 509, 200);
LobotSerialServoMove(9, 496, 200);
LobotSerialServoMove(10, 553, 200);
delay_ms(205);

}






void new_taizuotui(void)
{
// #1 P503 #2 P506 #3 P499 #4 P506 #5 P500 #6 P512 #7 P499 #8 P576 #9 P578 #10 P502
LobotSerialServoMove(1, 503, 200);
LobotSerialServoMove(2, 506, 200);
LobotSerialServoMove(3, 499, 200);
LobotSerialServoMove(4, 506, 200);
LobotSerialServoMove(5, 500, 200);
LobotSerialServoMove(6, 512, 200);
LobotSerialServoMove(7, 499, 200);
LobotSerialServoMove(8, 576, 200);
LobotSerialServoMove(9, 578, 200);
LobotSerialServoMove(10, 502, 200);
delay_ms(205);

// #1 P503 #2 P505 #3 P654 #4 P506 #5 P371 #6 P507 #7 P499 #8 P576 #9 P578 #10 P456
LobotSerialServoMove(1, 503, 200);
LobotSerialServoMove(2, 505, 200);
LobotSerialServoMove(3, 654, 200);
LobotSerialServoMove(4, 506, 200);
LobotSerialServoMove(5, 371, 200);
LobotSerialServoMove(6, 507, 200);
LobotSerialServoMove(7, 499, 200);
LobotSerialServoMove(8, 576, 200);
LobotSerialServoMove(9, 578, 200);
LobotSerialServoMove(10, 456, 200);
delay_ms(205);

// #1 P503 #2 P506 #3 P655 #4 P506 #5 P372 #6 P507 #7 P499 #8 P576 #9 P578 #10 P457
LobotSerialServoMove(1, 503, 3000);
LobotSerialServoMove(2, 506, 3000);
LobotSerialServoMove(3, 655, 3000);
LobotSerialServoMove(4, 506, 3000);
LobotSerialServoMove(5, 372, 3000);
LobotSerialServoMove(6, 507, 3000);
LobotSerialServoMove(7, 499, 3000);
LobotSerialServoMove(8, 576, 3000);
LobotSerialServoMove(9, 578, 3000);
LobotSerialServoMove(10, 457, 3000);
delay_ms(3005);

// #1 P503 #2 P506 #3 P498 #4 P506 #5 P497 #6 P509 #7 P499 #8 P578 #9 P572 #10 P498
LobotSerialServoMove(1, 503, 200);
LobotSerialServoMove(2, 506, 200);
LobotSerialServoMove(3, 498, 200);
LobotSerialServoMove(4, 506, 200);
LobotSerialServoMove(5, 497, 200);
LobotSerialServoMove(6, 509, 200);
LobotSerialServoMove(7, 499, 200);
LobotSerialServoMove(8, 578, 200);
LobotSerialServoMove(9, 572, 200);
LobotSerialServoMove(10, 498, 200);
delay_ms(205);

// #1 P503 #2 P507 #3 P497 #4 P506 #5 P549 #6 P506 #7 P526 #8 P504 #9 P503 #10 P499
LobotSerialServoMove(1, 503, 200);
LobotSerialServoMove(2, 507, 200);
LobotSerialServoMove(3, 497, 200);
LobotSerialServoMove(4, 506, 200);
LobotSerialServoMove(5, 549, 200);
LobotSerialServoMove(6, 506, 200);
LobotSerialServoMove(7, 526, 200);
LobotSerialServoMove(8, 504, 200);
LobotSerialServoMove(9, 503, 200);
LobotSerialServoMove(10, 499, 200);
delay_ms(205);

}

void new_zuozhuan(void)
{
// #1 P498 #2 P508 #3 P490 #4 P536 #5 P578 #6 P506 #7 P530 #8 P503 #9 P504 #10 P515
LobotSerialServoMove(1, 498, 300);
LobotSerialServoMove(2, 508, 300);
LobotSerialServoMove(3, 490, 300);
LobotSerialServoMove(4, 536, 300);
LobotSerialServoMove(5, 578, 300);
LobotSerialServoMove(6, 506, 300);
LobotSerialServoMove(7, 530, 300);
LobotSerialServoMove(8, 503, 300);
LobotSerialServoMove(9, 504, 300);
LobotSerialServoMove(10, 515, 300);
delay_ms(305);

// #1 P495 #2 P508 #3 P490 #4 P535 #5 P572 #6 P503 #7 P516 #8 P566 #9 P580 #10 P510
LobotSerialServoMove(1, 495, 300);
LobotSerialServoMove(2, 508, 300);
LobotSerialServoMove(3, 490, 300);
LobotSerialServoMove(4, 535, 300);
LobotSerialServoMove(5, 572, 300);
LobotSerialServoMove(6, 503, 300);
LobotSerialServoMove(7, 516, 300);
LobotSerialServoMove(8, 566, 300);
LobotSerialServoMove(9, 580, 300);
LobotSerialServoMove(10, 510, 300);
delay_ms(305);

// #1 P495 #2 P388 #3 P485 #4 P542 #5 P513 #6 P502 #7 P520 #8 P564 #9 P576 #10 P549
LobotSerialServoMove(1, 495, 300);
LobotSerialServoMove(2, 388, 300);
LobotSerialServoMove(3, 485, 300);
LobotSerialServoMove(4, 542, 300);
LobotSerialServoMove(5, 513, 300);
LobotSerialServoMove(6, 502, 300);
LobotSerialServoMove(7, 520, 300);
LobotSerialServoMove(8, 564, 300);
LobotSerialServoMove(9, 576, 300);
LobotSerialServoMove(10, 549, 300);
delay_ms(305);

// #1 P495 #2 P392 #3 P536 #4 P537 #5 P393 #6 P504 #7 P518 #8 P504 #9 P485 #10 P616
LobotSerialServoMove(1, 495, 300);
LobotSerialServoMove(2, 392, 300);
LobotSerialServoMove(3, 536, 300);
LobotSerialServoMove(4, 537, 300);
LobotSerialServoMove(5, 393, 300);
LobotSerialServoMove(6, 504, 300);
LobotSerialServoMove(7, 518, 300);
LobotSerialServoMove(8, 504, 300);
LobotSerialServoMove(9, 485, 300);
LobotSerialServoMove(10, 600, 300);
delay_ms(305);

// #1 P496 #2 P392 #3 P532 #4 P454 #5 P379 #6 P392 #7 P447 #8 P459 #9 P482 #10 P619
LobotSerialServoMove(1, 496, 300);
LobotSerialServoMove(2, 392, 300);
LobotSerialServoMove(3, 532, 300);
LobotSerialServoMove(4, 454, 300);
LobotSerialServoMove(5, 379, 300);
LobotSerialServoMove(6, 392, 300);
LobotSerialServoMove(7, 447, 300);
LobotSerialServoMove(8, 459, 300);
LobotSerialServoMove(9, 482, 300);
LobotSerialServoMove(10, 600, 300);
delay_ms(305);

// #1 P495 #2 P523 #3 P567 #4 P462 #5 P374 #6 P394 #7 P466 #8 P466 #9 P482 #10 P630
LobotSerialServoMove(1, 495, 300);
LobotSerialServoMove(2, 523, 300);
LobotSerialServoMove(3, 567, 300);
LobotSerialServoMove(4, 462, 300);
LobotSerialServoMove(5, 374, 300);
LobotSerialServoMove(6, 394, 300);
LobotSerialServoMove(7, 466, 300);
LobotSerialServoMove(8, 466, 300);
LobotSerialServoMove(9, 482, 300);
LobotSerialServoMove(10, 630, 300);
delay_ms(305);

// #1 P498 #2 P508 #3 P490 #4 P536 #5 P578 #6 P506 #7 P530 #8 P503 #9 P504 #10 P515
LobotSerialServoMove(1, 498, 300);
LobotSerialServoMove(2, 508, 300);
LobotSerialServoMove(3, 490, 300);
LobotSerialServoMove(4, 536, 300);
LobotSerialServoMove(5, 578, 300);
LobotSerialServoMove(6, 506, 300);
LobotSerialServoMove(7, 530, 300);
LobotSerialServoMove(8, 503, 300);
LobotSerialServoMove(9, 504, 300);
LobotSerialServoMove(10, 515, 300);
delay_ms(305);


}


//void new_youzhuan(void)
//{
//// #1 P499 #2 P501 #3 P505 #4 P500 #5 P499 #6 P502 #7 P539 #8 P444 #9 P427 #10 P551
//LobotSerialServoMove(1, 499, 200);
//LobotSerialServoMove(2, 501, 200);
//LobotSerialServoMove(3, 505, 200);
//LobotSerialServoMove(4, 500, 200);
//LobotSerialServoMove(5, 499, 200);
//LobotSerialServoMove(6, 502, 200);
//LobotSerialServoMove(7, 539, 200);
//LobotSerialServoMove(8, 444, 200);
//LobotSerialServoMove(9, 427, 200);
//LobotSerialServoMove(10, 551, 200);
//delay_ms(205);

//// #1 P567 #2 P501 #3 P508 #4 P501 #5 P500 #6 P501 #7 P540 #8 P448 #9 P431 #10 P555
//LobotSerialServoMove(1, 567, 200);
//LobotSerialServoMove(2, 501, 200);
//LobotSerialServoMove(3, 508, 200);
//LobotSerialServoMove(4, 501, 200);
//LobotSerialServoMove(5, 500, 200);
//LobotSerialServoMove(6, 501, 200);
//LobotSerialServoMove(7, 540, 200);
//LobotSerialServoMove(8, 448, 200);
//LobotSerialServoMove(9, 431, 200);
//LobotSerialServoMove(10, 555, 200);
//delay_ms(205);

//// #1 P565 #2 P501 #3 P507 #4 P504 #5 P499 #6 P535 #7 P578 #8 P531 #9 P508 #10 P553
//LobotSerialServoMove(1, 565, 200);
//LobotSerialServoMove(2, 501, 200);
//LobotSerialServoMove(3, 507, 200);
//LobotSerialServoMove(4, 504, 200);
//LobotSerialServoMove(5, 499, 200);
//LobotSerialServoMove(6, 535, 200);
//LobotSerialServoMove(7, 578, 200);
//LobotSerialServoMove(8, 531, 200);
//LobotSerialServoMove(9, 508, 200);
//LobotSerialServoMove(10, 553, 200);
//delay_ms(205);

//// #1 P559 #2 P501 #3 P502 #4 P505 #5 P499 #6 P533 #7 P581 #8 P599 #9 P563 #10 P553
//LobotSerialServoMove(1, 559, 200);
//LobotSerialServoMove(2, 501, 200);
//LobotSerialServoMove(3, 502, 200);
//LobotSerialServoMove(4, 505, 200);
//LobotSerialServoMove(5, 499, 200);
//LobotSerialServoMove(6, 533, 200);
//LobotSerialServoMove(7, 581, 200);
//LobotSerialServoMove(8, 599, 200);
//LobotSerialServoMove(9, 563, 200);
//LobotSerialServoMove(10, 553, 200);
//delay_ms(205);

//// #1 P511 #2 P500 #3 P505 #4 P506 #5 P509 #6 P511 #7 P566 #8 P504 #9 P502 #10 P560
//LobotSerialServoMove(1, 511, 200);
//LobotSerialServoMove(2, 500, 200);
//LobotSerialServoMove(3, 505, 200);
//LobotSerialServoMove(4, 506, 200);
//LobotSerialServoMove(5, 509, 200);
//LobotSerialServoMove(6, 511, 200);
//LobotSerialServoMove(7, 566, 200);
//LobotSerialServoMove(8, 504, 200);
//LobotSerialServoMove(9, 502, 200);
//LobotSerialServoMove(10, 560, 200);
//delay_ms(205);

//}
void new_youzhuan(void)
{
// #1 P504 #2 P505 #3 P498 #4 P500 #5 P503 #6 P495 #7 P501 #8 P448 #9 P444 #10 P536
LobotSerialServoMove(1, 504, 300);
LobotSerialServoMove(2, 505, 300);
LobotSerialServoMove(3, 498, 300);
LobotSerialServoMove(4, 500, 300);
LobotSerialServoMove(5, 503, 300);
LobotSerialServoMove(6, 495, 300);
LobotSerialServoMove(7, 501, 300);
LobotSerialServoMove(8, 448, 300);
LobotSerialServoMove(9, 444, 300);
LobotSerialServoMove(10, 536, 300);
delay_ms(305);

// #1 P570 #2 P506 #3 P498 #4 P500 #5 P505 #6 P496 #7 P501 #8 P454 #9 P447 #10 P537
LobotSerialServoMove(1, 570, 300);
LobotSerialServoMove(2, 506, 300);
LobotSerialServoMove(3, 498, 300);
LobotSerialServoMove(4, 500, 300);
LobotSerialServoMove(5, 505, 300);
LobotSerialServoMove(6, 496, 300);
LobotSerialServoMove(7, 501, 300);
LobotSerialServoMove(8, 454, 300);
LobotSerialServoMove(9, 447, 300);
LobotSerialServoMove(10, 537, 300);
delay_ms(305);

// #1 P561 #2 P505 #3 P498 #4 P501 #5 P504 #6 P497 #7 P535 #8 P489 #9 P506 #10 P537
LobotSerialServoMove(1, 561, 300);
LobotSerialServoMove(2, 505, 300);
LobotSerialServoMove(3, 498, 300);
LobotSerialServoMove(4, 501, 300);
LobotSerialServoMove(5, 504, 300);
LobotSerialServoMove(6, 497, 300);
LobotSerialServoMove(7, 535, 300);
LobotSerialServoMove(8, 489, 300);
LobotSerialServoMove(9, 506, 300);
LobotSerialServoMove(10, 537, 300);
delay_ms(305);

// #1 P538 #2 P506 #3 P502 #4 P508 #5 P500 #6 P491 #7 P538 #8 P566 #9 P566 #10 P543
LobotSerialServoMove(1, 538, 400);
LobotSerialServoMove(2, 506, 400);
LobotSerialServoMove(3, 502, 400);
LobotSerialServoMove(4, 508, 400);
LobotSerialServoMove(5, 500, 400);
LobotSerialServoMove(6, 491, 400);
LobotSerialServoMove(7, 538, 400);
LobotSerialServoMove(8, 566, 400);
LobotSerialServoMove(9, 566, 400);
LobotSerialServoMove(10, 543, 400);
delay_ms(405);

// #1 P474 #2 P506 #3 P502 #4 P509 #5 P501 #6 P489 #7 P544 #8 P564 #9 P596 #10 P543
LobotSerialServoMove(1, 474, 600);
LobotSerialServoMove(2, 506, 600);
LobotSerialServoMove(3, 502, 600);
LobotSerialServoMove(4, 509, 600);
LobotSerialServoMove(5, 501, 600);
LobotSerialServoMove(6, 489, 600);
LobotSerialServoMove(7, 544, 600);
LobotSerialServoMove(8, 564, 600);
LobotSerialServoMove(9, 596, 600);
LobotSerialServoMove(10, 543, 600);
delay_ms(605);

// #1 P494 #2 P506 #3 P503 #4 P508 #5 P501 #6 P493 #7 P500 #8 P504 #9 P504 #10 P508
LobotSerialServoMove(1, 494, 600);
LobotSerialServoMove(2, 506, 600);
LobotSerialServoMove(3, 503, 600);
LobotSerialServoMove(4, 508, 600);
LobotSerialServoMove(5, 501, 600);
LobotSerialServoMove(6, 493, 600);
LobotSerialServoMove(7, 500, 600);
LobotSerialServoMove(8, 504, 600);
LobotSerialServoMove(9, 504, 600);
LobotSerialServoMove(10, 508, 600);
delay_ms(605);

}

