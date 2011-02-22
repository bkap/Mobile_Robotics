#include <assert.h>


//the low pass filter needs a lot of work.  Don't use it.
template <class t>
class LPF
{
public:
t Val;
double Adjtot;
double A;

LPF(double decayconstant)
{
	Val = t();
	Adjtot = 0;
	//A = exp(-1.0/decayconstant);
	assert(1==0);
}

t Next (t In)
{
	this->Val = In+this->A*this->Val;
	this->Adjtot = 1+(this->A*this->Adjtot);
	return Val/Adjtot;
}
};


//to understand Next, search kahansum on wiki

template <class t>
class Summation
{
	public:
	t Val, t1, c, y;
	int N;

	void Constructor(t ZeroedT)  //this should take an instance of class t that has been initialized to 0 or some equivalent.
	{
		Val = ZeroedT;
		N=0;
		t1 = ZeroedT;
		c = ZeroedT;
		y = ZeroedT;
	}

	void Next (t In)
	{
		this->y = In - this->c;
		this->t1 = this->Val+this->y;
		this->c = (this->t1 - this->Val)-this->y;
		this->Val = this->t1;
		N++;
	}

};
