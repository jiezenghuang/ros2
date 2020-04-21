#ifndef _STATE_MACHINE_HPP_
#define _STATE_MACHINE_HPP_

#include <memory>
#include <functional>
#include <thread>
#include <map>
#include <exception>
#include <sstream>

class Alphabet
{
    public:
    explicit Alphabet(int k, void* v = nullptr)
        : key(k), value(v)
    {}
    virtual ~Alphabet() {}

    int key;
    void* value;
};

class State
{
    public:
    explicit State(int key, std::function<std::shared_ptr<Alphabet>(std::shared_ptr<Alphabet>)> exec)
        : key_(key), execute_(exec)
    {
        transitions_.clear();
    }

    virtual ~State() {}

    int get_key()
    {
        return key_;
    }

    void add_transition(int alphabet, int state)
    {
        transitions_[alphabet] = state;
    }

    int get_transition_state(int alphabet)
    {
        if(transitions_.find(alphabet) == transitions_.end())
        {
            std::ostringstream error;
            error << "transition " << alphabet << " is undefine in state " << key_ << ".";
            throw std::range_error(error.str());
        }            
        return transitions_[alphabet];
    }

    std::shared_ptr<Alphabet> execute(std::shared_ptr<Alphabet> alphabet)
    {
        if(execute_ != nullptr)
            return execute_(alphabet);
    }

    protected:    
    int key_;

    private:
    std::map<int, int> transitions_;
    std::function<std::shared_ptr<Alphabet>(std::shared_ptr<Alphabet>)> execute_;

    State(const State& s) {}
    State& operator=(const State& )
    {
        return *this;
    }
};

class StateMachine
{
    public:
    StateMachine()
        : current_(nullptr) 
    {
        states_.clear();
    }
    ~StateMachine() {}

    void add_state(std::shared_ptr<State> state)
    {
        states_[state->get_key()] = state;
    }

    void add_state_transition(int key, int alphabet, int state)
    {
        get_next_state(key)->add_transition(alphabet, state);
    }

    void start(int state)
    {
        current_ = get_next_state(state);
        std::thread process_thread(&StateMachine::process, this);
        process_thread.detach();
    }     

    void stop()
    {
        current_ = nullptr;   
    }

    int get_current_state()
    {
        return current_->get_key();
    }

    private:
    std::shared_ptr<State> current_;
    std::map<int, std::shared_ptr<State>> states_;

    void process()
    {
        std::shared_ptr<Alphabet> alphabet = nullptr;
        while(current_ != nullptr)
        {
            alphabet = current_->execute(alphabet);
            if(alphabet == nullptr)
                break;
            current_ = get_next_state(current_->get_transition_state(alphabet->key));      
        }
    }

    std::shared_ptr<State> get_next_state(int key)
    {
        if(states_.find(key) == states_.end())
        {
            std::ostringstream error;
            error << "state " << key << " is undefine in state machine.";
            throw std::range_error(error.str());
        }
        return states_[key]; 
    }  

    StateMachine(const StateMachine& ) {}
    StateMachine& operator=(const StateMachine& )
    {
        return *this;
    }
};

#endif