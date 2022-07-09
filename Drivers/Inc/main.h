/*
 * main.h
 *
 *  Created on: 09-Jul-2022
 *      Author: Vaishnavi
 */

#ifndef MAIN_H_
#define MAIN_H_


template<typename T>
class API
{
protected:
  T *pReg;

public:
  virtual void ClockControl(bool En_Dis) = 0;
  virtual void DeInit() = 0;
  void Init();
};


#endif /* MAIN_H_ */
