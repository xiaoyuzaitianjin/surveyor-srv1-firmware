#include "picoc.h"

/* the table of global definitions */
struct Table GlobalTable;
struct TableEntry *GlobalHashTable[GLOBAL_TABLE_SIZE];

/* the stack */
struct StackFrame *TopStackFrame = NULL;


/* initialise the variable system */
void VariableInit()
{
    TableInitTable(&GlobalTable, &GlobalHashTable[0], GLOBAL_TABLE_SIZE, TRUE);
    TopStackFrame = NULL;
}

/* allocate some memory, either on the heap or the stack and check if we've run out */
void *VariableAlloc(struct ParseState *Parser, int Size, int OnHeap)
{
    void *NewValue;
    
    if (OnHeap)
        NewValue = HeapAlloc(Size);
    else
        NewValue = HeapAllocStack(Size);
    
    if (NewValue == NULL)
        ProgramFail(Parser, "out of memory");
    
#ifdef DEBUG_HEAP
    if (!OnHeap)
        printf("pushing %d at 0x%lx\n", Size, (unsigned long)NewValue);
#endif
        
    return NewValue;
}

/* allocate a value either on the heap or the stack using space dependent on what type we want */
struct Value *VariableAllocValueAndData(struct ParseState *Parser, int DataSize, int IsLValue, struct Value *LValueFrom, int OnHeap)
{
    struct Value *NewValue = VariableAlloc(Parser, sizeof(struct Value) + DataSize, OnHeap);
    NewValue->Val = (union AnyValue *)((void *)NewValue + sizeof(struct Value));
    NewValue->ValOnHeap = OnHeap;
    NewValue->ValOnStack = !OnHeap;
    NewValue->IsLValue = IsLValue;
    NewValue->LValueFrom = LValueFrom;
    
    return NewValue;
}

/* allocate a value given its type */
struct Value *VariableAllocValueFromType(struct ParseState *Parser, struct ValueType *Typ, int IsLValue, struct Value *LValueFrom)
{
    int Size = TypeSize(Typ, Typ->ArraySize);
    struct Value *NewValue = VariableAllocValueAndData(Parser, Size, IsLValue, LValueFrom, FALSE);
    assert(Size > 0 || Typ == &VoidType);
    NewValue->Typ = Typ;
    if (Typ->Base == TypeArray)
    {
        NewValue->Val->Array.Size = Typ->ArraySize;
        NewValue->Val->Array.Data = (void *)NewValue->Val;
    }
    
    return NewValue;
}

/* allocate a value either on the heap or the stack and copy its value */
struct Value *VariableAllocValueAndCopy(struct ParseState *Parser, struct Value *FromValue, int OnHeap)
{
    int CopySize = TypeSizeValue(FromValue);
    struct Value *NewValue = VariableAllocValueAndData(Parser, CopySize, FromValue->IsLValue, FromValue->LValueFrom, OnHeap);
    NewValue->Typ = FromValue->Typ;
    memcpy(NewValue->Val, FromValue->Val, CopySize);
    return NewValue;
}

/* allocate a value either on the heap or the stack from an existing AnyValue and type */
struct Value *VariableAllocValueFromExistingData(struct ParseState *Parser, struct ValueType *Typ, union AnyValue *FromValue, int IsLValue, struct Value *LValueFrom)
{
    struct Value *NewValue = VariableAlloc(Parser, sizeof(struct Value), FALSE);
    NewValue->Typ = Typ;
    NewValue->Val = FromValue;
    NewValue->ValOnHeap = FALSE;
    NewValue->ValOnStack = FALSE;
    NewValue->IsLValue = IsLValue;
    NewValue->LValueFrom = LValueFrom;
    
    return NewValue;
}

/* allocate a value either on the heap or the stack from an existing Value, sharing the value */
struct Value *VariableAllocValueShared(struct ParseState *Parser, struct Value *FromValue)
{
    return VariableAllocValueFromExistingData(Parser, FromValue->Typ, FromValue->Val, FromValue->IsLValue, FromValue->IsLValue ? FromValue : NULL);
}

/* define a variable */
void VariableDefine(struct ParseState *Parser, char *Ident, struct Value *InitValue)
{
    if (!TableSet((TopStackFrame == NULL) ? &GlobalTable : &TopStackFrame->LocalTable, Ident, VariableAllocValueAndCopy(Parser, InitValue, TopStackFrame == NULL)))
        ProgramFail(Parser, "'%s' is already defined", Ident);
}

/* check if a variable with a given name is defined */
int VariableDefined(const char *Ident)
{
    struct Value *FoundValue;
    
    if (TopStackFrame == NULL || !TableGet(&TopStackFrame->LocalTable, Ident, &FoundValue))
    {
        if (!TableGet(&GlobalTable, Ident, &FoundValue))
            return FALSE;
    }

    return TRUE;
}

/* get the value of a variable. must be defined */
void VariableGet(struct ParseState *Parser, const char *Ident, struct Value **LVal)
{
    if (TopStackFrame == NULL || !TableGet(&TopStackFrame->LocalTable, Ident, LVal))
    {
        if (!TableGet(&GlobalTable, Ident, LVal))
            ProgramFail(Parser, "'%s' is undefined", Ident);
    }
}

/* define a global variable shared with a platform global */
void VariableDefinePlatformVar(struct ParseState *Parser, char *Ident, struct ValueType *Typ, union AnyValue *FromValue, int IsWritable)
{
    struct Value *SomeValue = VariableAllocValueAndData(NULL, (Typ->Base == TypeArray) ? sizeof(struct ArrayValue) : 0, IsWritable, NULL, TRUE);
    SomeValue->Typ = Typ;
    if (Typ->Base != TypeArray)
        SomeValue->Val = FromValue;
    else
    { /* define an array */
        SomeValue->Val->Array.Size = Typ->ArraySize;
        SomeValue->Val->Array.Data = FromValue;
    }
    
    VariableDefine(Parser, TableStrRegister(Ident), SomeValue);
}

/* free and/or pop the top value off the stack. Var must be the top value on the stack! */
void VariableStackPop(struct ParseState *Parser, struct Value *Var)
{
    int Success;
    
#ifdef DEBUG_HEAP
    if (Var->ValOnStack)
        printf("popping %d at 0x%lx\n", sizeof(struct Value) + VariableSizeValue(Var), (unsigned long)Var);
#endif
        
    if (Var->ValOnHeap)
    { 
        HeapFree(Var->Val);
        Success = HeapPopStack(Var, sizeof(struct Value));                       /* free from heap */
    }
    else if (Var->ValOnStack)
        Success = HeapPopStack(Var, sizeof(struct Value) + TypeSizeValue(Var));  /* free from stack */
    else
        Success = HeapPopStack(Var, sizeof(struct Value));                       /* value isn't our problem */
        
    if (!Success)
        ProgramFail(Parser, "stack underrun");
}

/* add a stack frame when doing a function call */
void VariableStackFrameAdd(struct ParseState *Parser, int NumParams)
{
    struct StackFrame *NewFrame;
    
    HeapPushStackFrame();
    NewFrame = HeapAllocStack(sizeof(struct StackFrame) + sizeof(struct Value *) * NumParams);
    if (NewFrame == NULL)
        ProgramFail(Parser, "out of memory");
        
    NewFrame->ReturnParser = *Parser;
    NewFrame->Parameter = (NumParams > 0) ? ((void *)NewFrame + sizeof(struct StackFrame)) : NULL;
    TableInitTable(&NewFrame->LocalTable, &NewFrame->LocalHashTable[0], LOCAL_TABLE_SIZE, FALSE);
    NewFrame->PreviousStackFrame = TopStackFrame;
    TopStackFrame = NewFrame;
}

/* remove a stack frame */
void VariableStackFramePop(struct ParseState *Parser)
{
    if (TopStackFrame == NULL)
        ProgramFail(Parser, "stack is empty - can't go back");
        
    *Parser = TopStackFrame->ReturnParser;
    TopStackFrame = TopStackFrame->PreviousStackFrame;
    HeapPopStackFrame();
}