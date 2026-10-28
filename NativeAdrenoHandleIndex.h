/* SPDX-License-Identifier: MIT */
#ifndef NATIVE_ADRENO_HANDLE_INDEX_H
#define NATIVE_ADRENO_HANDLE_INDEX_H
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <IOKit/IOLib.h>
/* Handle -> slot of a fixed table whose handles are never 0: open addressing over twice the slots with linear probing, deletion by
 * backward shift (no tombstones), so a lookup is a few probes instead of a walk over every slot. i567: the submit path looked each IB's
 * handle up three times by walking a 4096-entry table, and the table had to grow to 16384 entries. */
/* The cells grow with the keys (from 64, doubling at half load, up to twice Slots), so an index over a few objects costs a few cells: every
 * process that uses the GPU has two of them. */
template<size_t Slots> class NativeAdrenoHandleIndex {
    enum : size_t {MaxSize=Slots*2,FirstSize=64};
    static_assert(Slots && !(MaxSize&(MaxSize-1)) && FirstSize<=MaxSize,"power of two");
    struct Cell {uint64_t key;uint32_t value;};
    Cell *cells_=nullptr;
    size_t size_=0,count_=0;
    size_t home(uint64_t key) const {return (size_t)((key*UINT64_C(0x9e3779b97f4a7c15))>>40)&(size_-1);}
    void place(uint64_t key,uint32_t value)
    {
        size_t i=home(key);
        while(cells_[i].key && cells_[i].key!=key)i=(i+1)&(size_-1);
        if(!cells_[i].key)++count_;
        cells_[i]={key,value};
    }
    /* Twice the cells, every key placed again; false (nothing changed) without memory or at the largest size. */
    bool grow()
    {
        if(size_>=MaxSize)return false;
        Cell *old=cells_;const size_t oldSize=size_;
        Cell *made=IONewData(Cell,oldSize*2);
        if(!made)return false;
        memset(made,0,sizeof(Cell)*oldSize*2);
        cells_=made;size_=oldSize*2;count_=0;
        for(size_t i=0;i<oldSize;++i)if(old[i].key)place(old[i].key,old[i].value);
        IODeleteData(old,Cell,oldSize);
        return true;
    }
public:
    bool allocate()
    {
        release();
        cells_=IONewData(Cell,FirstSize);
        if(!cells_)return false;
        memset(cells_,0,sizeof(Cell)*FirstSize);size_=FirstSize;count_=0;
        return true;
    }
    void release() {if(cells_)IODeleteData(cells_,Cell,size_);cells_=nullptr;size_=0;count_=0;}
    /* UINT32_MAX: not present. */
    uint32_t find(uint64_t key) const
    {
        if(!cells_ || !key)return UINT32_MAX;
        for(size_t i=home(key),n=0;n<size_;i=(i+1)&(size_-1),++n) {
            if(!cells_[i].key)return UINT32_MAX;
            if(cells_[i].key==key)return cells_[i].value;
        }
        return UINT32_MAX;
    }
    /* At most Slots keys are ever present, so at the largest size a free cell always exists. Below it the cells double once half are
     * used; a failed doubling keeps the current cells, which still have a free one (the load stays at most Slots of 2*Slots). */
    void insert(uint64_t key,uint32_t value)
    {
        if(!cells_ || !key)return;
        if((count_+1)*2>size_ && !grow() && count_+1>=size_)return;
        place(key,value);
    }
    void erase(uint64_t key)
    {
        if(!cells_ || !key)return;
        const size_t mask=size_-1;
        size_t i=home(key);
        for(size_t n=0;;i=(i+1)&mask,++n) {
            if(n==size_ || !cells_[i].key)return;
            if(cells_[i].key==key)break;
        }
        /* Pull back each later cell of the run whose home is not cyclically inside (i, j]. */
        for(size_t j=i;;) {
            j=(j+1)&mask;
            if(!cells_[j].key)break;
            const size_t k=home(cells_[j].key);
            if(i<=j?(i<k && k<=j):(i<k || k<=j))continue;
            cells_[i]=cells_[j];i=j;
        }
        cells_[i]={0,0};
        --count_;
    }
};
#endif
