// Help you build your state machine, it's real-time compitable.
#ifndef STAMACH_H
#define STAMACH_H
#include<string>
#include<cstdio>
#include<memory.h>

using namespace std;

namespace StaMach
{


#define NAME_LENGTH 6

template <class OrderEnum, class StateEnum, int OrderNum, int StateNum >
class CStaMach
{
public:
    CStaMach();
    ~CStaMach();

    void SetOffset(int orderoffset,int stateoffset);
    // Usage of StaMach Class
    // First the template define demnsions and type of the state machine

    // This function define the start point of the state transition
    void SetInitialState(StateEnum state);

    // When no command comes, this will be the default order
    void SetDefaultOrder(OrderEnum order);

    void AddOrder(OrderEnum order);
    void AddState(StateEnum state);
    void AddTransition(StateEnum state_in,OrderEnum order,StateEnum state_out);

    void AddOrderWithState(OrderEnum order,StateEnum state);

    // normal usage
    void SetOrder(OrderEnum order);
    StateEnum GetNextState();
    bool IfFirstCycle();
    void SetNextState(StateEnum state);

    // special usage
    // Force the current state and next state be a designated state
    void ForceState(StateEnum state);

    char const* GetTransitionDesciption();
    void SetNames(char ordernames[][NAME_LENGTH],char statenames[][NAME_LENGTH]);
    void PrintStateMachine();



    StateEnum m_TransitionMatrix[OrderNum][StateNum];
private:
    StateEnum m_CurrentState=(StateEnum)0;
public:
    StateEnum m_NextState=(StateEnum)0;
    OrderEnum m_CurrentOrder=(OrderEnum)0;
private:
    char m_TransitionDesciption[200];

    int m_OrderOffset=0;
    int m_StateOffset=0;

    char (*m_OrderName)[NAME_LENGTH];
    char (*m_StateName)[NAME_LENGTH];

};


template <class OrderEnum, class StateEnum, int OrderNum, int StateNum>
void CStaMach<OrderEnum,StateEnum,OrderNum,StateNum>::SetOrder(OrderEnum order)
{
    m_CurrentOrder=order;

    m_NextState=m_TransitionMatrix[m_CurrentOrder-m_OrderOffset][m_NextState-m_StateOffset];

//    if(order!=m_OrderOffset)
//        printf("%d %d\n%d %d\n%d %d\n%d\n",
//               m_CurrentOrder,m_StateOffset,
//               m_OrderOffset,m_StateOffset,
//               m_CurrentOrder-m_OrderOffset,m_NextState-m_StateOffset,
//               m_NextState);
}


template <class OrderEnum, class StateEnum, int OrderNum, int StateNum>
void CStaMach<OrderEnum,StateEnum,OrderNum,StateNum>::SetNextState(StateEnum state)
{
    m_NextState=state;
}


template <class OrderEnum, class StateEnum, int OrderNum, int StateNum>
StateEnum CStaMach<OrderEnum,StateEnum,OrderNum,StateNum>::GetNextState()
{
    //m_CurrentState=m_NextState;
//    printf("GetNextState: %d\n",m_NextState);
    return m_NextState;
}

template <class OrderEnum, class StateEnum, int OrderNum, int StateNum>
void CStaMach<OrderEnum,StateEnum,OrderNum,StateNum>::SetOffset(int orderoffset, int stateoffset)
{
    m_OrderOffset=orderoffset;
    m_StateOffset=stateoffset;
    for(int i=0; i<OrderNum; i++)
    {
        for(int j=0; j<StateNum; j++)
        {
            m_TransitionMatrix[i][j]=StateEnum(stateoffset);

        }
    }
}

template <class OrderEnum, class StateEnum, int OrderNum, int StateNum>
bool CStaMach<OrderEnum,StateEnum,OrderNum,StateNum>::IfFirstCycle()
{
    if(m_CurrentState==m_NextState)
    {
        return false;
    }
    else
    {
        //becasue only when transition happens (this function retuens true)
        // the transition may be printed on the screen

        memset(m_TransitionDesciption,0,sizeof(m_TransitionDesciption));

        char order[50];
        char state[50];

        sprintf(order,"TRANSITION Order: %d %s,",
                m_CurrentOrder,
                m_OrderName[m_CurrentOrder-m_OrderOffset]);

        sprintf(state,"State has changed to %d %s from %d %s \n",
                m_NextState,
                m_StateName[m_NextState-m_StateOffset],
                m_CurrentState,
                m_StateName[m_CurrentState-m_StateOffset]);

        strcat(m_TransitionDesciption,order);
        strcat(m_TransitionDesciption,state);

        // set the current state to next state
        m_CurrentState=m_NextState;
        return true;
    }
}

template <class OrderEnum, class StateEnum, int OrderNum, int StateNum>
void CStaMach<OrderEnum,StateEnum,OrderNum,StateNum>::ForceState(StateEnum state)
{
    m_CurrentState=state;
    m_NextState=state;
}

template <class OrderEnum, class StateEnum, int OrderNum, int StateNum>
CStaMach<OrderEnum,StateEnum,OrderNum,StateNum>::CStaMach()
{
    for(int i=0; i<OrderNum; i++)
    {
        for(int j=0; j<StateNum; j++)
        {

        }
    }

}

template <class OrderEnum, class StateEnum, int OrderNum, int StateNum>
CStaMach<OrderEnum,StateEnum,OrderNum,StateNum>::~CStaMach()
{
    for(int i=0; i<OrderNum; i++)
    {
        for(int j=0; j<StateNum; j++)
        {

        }
    }

}

template <class OrderEnum, class StateEnum, int OrderNum, int StateNum>
char const* CStaMach<OrderEnum,StateEnum,OrderNum,StateNum>::GetTransitionDesciption()
{

    return m_TransitionDesciption;
}


template <class OrderEnum, class StateEnum, int OrderNum, int StateNum>
void CStaMach<OrderEnum,StateEnum,OrderNum,StateNum>::SetInitialState(StateEnum state)
{
    m_CurrentState=state;
    m_NextState=state;
}

template <class OrderEnum, class StateEnum, int OrderNum, int StateNum>
void CStaMach<OrderEnum,StateEnum,OrderNum,StateNum>::SetDefaultOrder(OrderEnum order)
{
    m_CurrentOrder=order;
}

template <class OrderEnum, class StateEnum, int OrderNum, int StateNum>
void CStaMach<OrderEnum,StateEnum,OrderNum,StateNum>::AddTransition(StateEnum state_in,OrderEnum order,StateEnum state_out)
{
    m_TransitionMatrix[order-m_OrderOffset][state_in-m_StateOffset]=state_out;
}

template <class OrderEnum, class StateEnum, int OrderNum, int StateNum>
void CStaMach<OrderEnum,StateEnum,OrderNum,StateNum>::AddOrder(OrderEnum order)
{
    //the order and the state should not be a sparse matrix, then this fucntion will work fine.
    for(int i=0; i<OrderNum; i++)
    {
        // add a row in the transition matrix
        // and each unit is the state of the column
        // this means this order will not change any state
        m_TransitionMatrix[order-m_OrderOffset][(StateEnum)i]=(StateEnum)(i+m_StateOffset);
    }
}

template <class OrderEnum, class StateEnum, int OrderNum, int StateNum>
void CStaMach<OrderEnum,StateEnum,OrderNum,StateNum>::AddOrderWithState(OrderEnum order,StateEnum state)
{
    for(int i=0; i<StateNum; i++)
    {
        // add a row in the transition matrix
        // and each unit is the state of the column
        // this means this order will not change any state
        m_TransitionMatrix[order-m_OrderOffset][(StateEnum)i]=state;
    }
}



template <class OrderEnum, class StateEnum, int OrderNum, int StateNum>
void CStaMach<OrderEnum,StateEnum,OrderNum,StateNum>::AddState(StateEnum state)
{
    for(int i=0; i<OrderNum; i++)
    {
        // add a column in the transition matrix
        // each column is the state of the column
        // this means in this state, the any order will not change it
        m_TransitionMatrix[(OrderEnum)i][state-m_StateOffset]=state;
    }

}

template <class OrderEnum, class StateEnum, int OrderNum, int StateNum>
void CStaMach<OrderEnum,StateEnum,OrderNum,StateNum>::SetNames(char ordernames[][NAME_LENGTH], char statenames[][NAME_LENGTH])
{
    // just do it for PrintStateMachine
    m_OrderName=ordernames;
    m_StateName=statenames;
}



template <class OrderEnum, class StateEnum, int OrderNum, int StateNum>
void CStaMach<OrderEnum,StateEnum,OrderNum,StateNum>::PrintStateMachine()
{

    for(int i=0;i<((NAME_LENGTH+4+1)*(StateNum+1));i++)
    {
        printf("#");
    }
    printf("\n");


    printf("STATE MACHINE MATRIX Offset O %d S %d\n",m_OrderOffset,m_StateOffset);

    for(int i=0;i<((NAME_LENGTH+4+1)*(StateNum+1));i++)
    {
        printf("_");
    }
    printf("\n");

    int cnt=NAME_LENGTH+4-3;
    for(int i=0;i<cnt;i++)
    {
        printf(" ");
    }
    printf("O\\S|");
    for(int j=0;j<StateNum;j++)
    {
        printf("%4d %s|",j+m_StateOffset,m_StateName[j]);
    }
    printf("\n");

    for(int i=0;i<((NAME_LENGTH+4+1)*(StateNum+1));i++)
    {
        if((i+1)%(NAME_LENGTH+5)==0)
        {
            printf("+");
        }
        else
        {
            printf("-");
        }

    }
    printf("\n");



    for(int i=0;i<OrderNum;i++)
    {
        printf("%4d %s|",i+m_OrderOffset,m_OrderName[i]);
        for(int j=0;j<StateNum;j++)
        {
            printf("%4d %s|",m_TransitionMatrix[i][j],m_StateName[m_TransitionMatrix[i][j]-m_StateOffset]);

        }
        printf("\n");

        for(int i=0;i<((NAME_LENGTH+5)*(StateNum+1));i++)
        {
            if((i+1)%(NAME_LENGTH+5)==0)
            {
                printf("+");
            }
            else
            {
                printf("-");
            }

        }
        printf("\n");
    }

    for(int i=0;i<((NAME_LENGTH+4+1)*(StateNum+1));i++)
    {
        printf("#");
    }
    printf("\n");

}

}










#endif
