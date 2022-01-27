#ifndef CALL_HANDER_H
#define CALL_HANDER_H


template<typename Useless>
class CallHandler;

template<typename R, typename ...Args>
class CallHandler <R(Args...)>
{
public:
    typedef R(*FunctionSignature)(Args..., void*);

    CallHandler() {}
    CallHandler(FunctionSignature f, void *data) : f_(f), data_(data) {}

public:
    R operator()(Args... args)
    {
        if (f_) return f_(args..., data_);
        return R();
    }

private:
    FunctionSignature f_;
    void *data_;
};


template<typename ...Args>
class CallHandler <void(Args...)>
{
public:
    typedef void(*FunctionSignature)(Args..., void*);

    CallHandler() {}
    CallHandler(FunctionSignature f, void *data) : f_(f), data_(data) {}

public:
    void operator()(Args... args)
    {
        if (f_) f_(args..., data_);
    }

private:
    FunctionSignature f_;
    void *data_;
};

#endif  // CALL_HANDER_H
