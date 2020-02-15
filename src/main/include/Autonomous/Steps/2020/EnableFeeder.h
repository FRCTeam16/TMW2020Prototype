#pragma once

#include "Robot.h"
#include "Autonomous/Step.h"

class EnableFeeder : public Step {
public:
    explicit EnableFeeder(bool enableFeeder) 
    : enableFeeder(enableFeeder) {
    }

    bool Run(std::shared_ptr<World> world) override {
        std::cout << "EnableFeeder(" << enableFeeder << ")\n";
        if (enableFeeder) {
            Robot::turret->StartFeeder();
        } else {
            Robot::turret->StopFeeder();
        }
        return true;
    }

private:
    bool enableFeeder;
};