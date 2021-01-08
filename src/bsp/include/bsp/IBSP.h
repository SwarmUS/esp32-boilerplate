#ifndef IBSP_H
#define IBSP_H

class IBSP {
public:
    virtual ~IBSP() = default;

    /**
     * @brief Initialise chip for usage. Should be called within the main.
     */
    virtual void initChip() = 0;
};

#endif // IBSP_H
