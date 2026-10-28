/* SPDX-License-Identifier: MIT */
/* Actual Vulkan/Turnip/KMD control flow over the software kernel peer. It runs no
 * shader: pending fences, retained native objects/views/IOVAs and error handling
 * are checked independently of rendering or hardware timing. */
#include <vulkan/vulkan.h>
#include "NativeAdrenoABI.h"
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <thread>
#include <vector>

#define REQUIRE(x) do {if (!(x)) {std::fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x);return 1;}} while(0)
#define OK(x) REQUIRE((x)==VK_SUCCESS)
#define LOAD(name) auto name=(PFN_##name)get(instance,#name);REQUIRE(name)

struct Peer {
    void (*hold)(int);
    std::atomic<bool> release{false},forced{false};
    std::thread worker;
    explicit Peer(void (*fn)(int)):hold(fn){hold(1);}
    void watchdog() {
        worker=std::thread([this]{
            auto end=std::chrono::steady_clock::now()+std::chrono::milliseconds(500);
            while(!release.load() && std::chrono::steady_clock::now()<end)
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            if(!release.load())forced=true;
            hold(0);
        });
    }
    void finish(){release=true;hold(0);if(worker.joinable())worker.join();}
    ~Peer(){finish();}
};

int main(int argc,char **argv) {
    if(argc!=3)return 2;
    const bool pressure=!std::strcmp(argv[2],"pressure"),busy=!std::strcmp(argv[2],"busy");
    const bool fenceError=!std::strcmp(argv[2],"fence-error"),freeError=!std::strcmp(argv[2],"free-error");
    const bool leaseError=!std::strcmp(argv[2],"lease-error"),viewError=!std::strcmp(argv[2],"view-error");
    const bool cleanupError=!std::strcmp(argv[2],"map-cleanup-error");
    const bool fatal=fenceError||freeError||leaseError||viewError||cleanupError;
    void *library=dlopen(argv[1],RTLD_NOW|RTLD_LOCAL);REQUIRE(library);
    auto get=(PFN_vkGetInstanceProcAddr)dlsym(library,"vk_icdGetInstanceProcAddr");REQUIRE(get);
    auto live=(unsigned(*)())dlsym(RTLD_DEFAULT,"host_gpu_live_objects");
    auto submissions=(unsigned(*)())dlsym(RTLD_DEFAULT,"host_gpu_submissions");
    auto successes=(unsigned(*)(uint32_t))dlsym(RTLD_DEFAULT,"host_gpu_successes");
    auto lastIova=(uint64_t(*)())dlsym(RTLD_DEFAULT,"host_gpu_last_map_iova");
    auto hold=(void(*)(int))dlsym(RTLD_DEFAULT,"host_gpu_hold_completion");
    auto inject=(void(*)(uint32_t,int,uint32_t))dlsym(RTLD_DEFAULT,"host_gpu_inject_failure");
    REQUIRE(live&&submissions&&successes&&lastIova&&hold&&inject);
    VkInstance instance=VK_NULL_HANDLE;
    LOAD(vkCreateInstance);
    VkApplicationInfo app={.sType=VK_STRUCTURE_TYPE_APPLICATION_INFO,.pApplicationName="deferred BO control",.apiVersion=VK_API_VERSION_1_1};
    VkInstanceCreateInfo ii={.sType=VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO,.pApplicationInfo=&app};
    OK(vkCreateInstance(&ii,nullptr,&instance));
    LOAD(vkEnumeratePhysicalDevices);LOAD(vkGetPhysicalDeviceMemoryProperties);LOAD(vkGetPhysicalDeviceQueueFamilyProperties);
    LOAD(vkCreateDevice);LOAD(vkAllocateMemory);LOAD(vkFreeMemory);LOAD(vkMapMemory);
    LOAD(vkCreateBuffer);LOAD(vkDestroyBuffer);LOAD(vkGetBufferMemoryRequirements);LOAD(vkBindBufferMemory);
    LOAD(vkGetDeviceQueue);LOAD(vkCreateCommandPool);LOAD(vkAllocateCommandBuffers);LOAD(vkBeginCommandBuffer);
    LOAD(vkCmdFillBuffer);LOAD(vkEndCommandBuffer);LOAD(vkQueueSubmit);LOAD(vkDeviceWaitIdle);
    LOAD(vkDestroyCommandPool);LOAD(vkDestroyDevice);LOAD(vkDestroyInstance);
    uint32_t count=1;VkPhysicalDevice physical;OK(vkEnumeratePhysicalDevices(instance,&count,&physical));REQUIRE(count==1);
    VkPhysicalDeviceMemoryProperties memory={};vkGetPhysicalDeviceMemoryProperties(physical,&memory);
    int type=-1;
    for(uint32_t i=0;i<memory.memoryTypeCount;i++)if((memory.memoryTypes[i].propertyFlags&(VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT|VK_MEMORY_PROPERTY_HOST_COHERENT_BIT))==(VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT|VK_MEMORY_PROPERTY_HOST_COHERENT_BIT)){type=(int)i;break;}
    REQUIRE(type>=0);
    VkQueueFamilyProperties families[8];count=8;vkGetPhysicalDeviceQueueFamilyProperties(physical,&count,families);
    int family=-1;for(uint32_t i=0;i<count;i++)if(families[i].queueFlags&VK_QUEUE_GRAPHICS_BIT){family=(int)i;break;}
    REQUIRE(family>=0);
    float priority=1;
    VkDeviceQueueCreateInfo qi={.sType=VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO,.queueFamilyIndex=(uint32_t)family,.queueCount=1,.pQueuePriorities=&priority};
    VkDeviceCreateInfo di={.sType=VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO,.queueCreateInfoCount=1,.pQueueCreateInfos=&qi};
    VkDevice device;OK(vkCreateDevice(physical,&di,nullptr,&device));
    VkQueue queue;vkGetDeviceQueue(device,(uint32_t)family,0,&queue);
    VkBufferCreateInfo bi={.sType=VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO,.size=4096,.usage=VK_BUFFER_USAGE_TRANSFER_DST_BIT,.sharingMode=VK_SHARING_MODE_EXCLUSIVE};
    VkBuffer buffer;OK(vkCreateBuffer(device,&bi,nullptr,&buffer));
    VkMemoryRequirements req;vkGetBufferMemoryRequirements(device,buffer,&req);REQUIRE(req.memoryTypeBits&(1u<<type));
    VkMemoryAllocateInfo ai={.sType=VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO,.allocationSize=req.size,.memoryTypeIndex=(uint32_t)type};
    VkDeviceMemory used;OK(vkAllocateMemory(device,&ai,nullptr,&used));OK(vkBindBufferMemory(device,buffer,used,0));
    ai.allocationSize=16384;
    std::vector<VkDeviceMemory> unused(pressure?257:1);
    uint64_t oldIova=0;
    for(auto &allocation:unused){OK(vkAllocateMemory(device,&ai,nullptr,&allocation));void *address=nullptr;OK(vkMapMemory(device,allocation,0,VK_WHOLE_SIZE,0,&address));if(!oldIova)oldIova=lastIova();}
    VkCommandPoolCreateInfo pi={.sType=VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO,.queueFamilyIndex=(uint32_t)family};
    VkCommandPool pool;OK(vkCreateCommandPool(device,&pi,nullptr,&pool));
    VkCommandBufferAllocateInfo ci={.sType=VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO,.commandPool=pool,.level=VK_COMMAND_BUFFER_LEVEL_PRIMARY,.commandBufferCount=1};
    VkCommandBuffer commands;OK(vkAllocateCommandBuffers(device,&ci,&commands));
    VkCommandBufferBeginInfo begin={.sType=VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};OK(vkBeginCommandBuffer(commands,&begin));
    vkCmdFillBuffer(commands,buffer,0,4096,0x5a5a5a5a);OK(vkEndCommandBuffer(commands));
    Peer peer(hold);
    unsigned beforeSubmit=submissions();
    VkSubmitInfo si={.sType=VK_STRUCTURE_TYPE_SUBMIT_INFO,.commandBufferCount=1,.pCommandBuffers=&commands};
    OK(vkQueueSubmit(queue,1,&si,VK_NULL_HANDLE));REQUIRE(submissions()>beforeSubmit);
    const unsigned beforeLive=live(),beforeFree=successes(NativeAdrenoMethodBoFree),beforeUnmap=successes(NativeAdrenoMethodBoCpuUnmap);
    if(fenceError)inject(NativeAdrenoMethodWaitFence,7,1); // fixture kIOReturnIOError, before mutation
    if(!pressure)peer.watchdog();
    for(unsigned i=0;i<(pressure?256u:1u);i++){
        vkFreeMemory(device,unused[i],nullptr);unused[i]=VK_NULL_HANDLE;
        REQUIRE(!peer.forced.load());
        REQUIRE(successes(NativeAdrenoMethodBoFree)==beforeFree);
        REQUIRE(successes(NativeAdrenoMethodBoCpuUnmap)==beforeUnmap);
    }
    REQUIRE(live()==beforeLive);
    VkDeviceMemory probe=VK_NULL_HANDLE,after=VK_NULL_HANDLE;
    if(cleanupError){
        inject(NativeAdrenoMethodBoMap,7,1);inject(NativeAdrenoMethodBoFree,2,1);
        VkDeviceMemory refused=VK_NULL_HANDLE;
        REQUIRE(vkAllocateMemory(device,&ai,nullptr,&refused)==VK_ERROR_DEVICE_LOST);
        REQUIRE(live()==beforeLive+1);
        REQUIRE(successes(NativeAdrenoMethodBoFree)==beforeFree);
        REQUIRE(successes(NativeAdrenoMethodBoCpuUnmap)==beforeUnmap);
    }
    if(!fatal && !pressure){OK(vkAllocateMemory(device,&ai,nullptr,&probe));REQUIRE(lastIova()!=oldIova);REQUIRE(successes(NativeAdrenoMethodBoFree)==beforeFree);}
    if(pressure){
        /* The deferred queue grows: the 257th free does not wait for the GPU either. */
        peer.watchdog();
        vkFreeMemory(device,unused.back(),nullptr);unused.back()=VK_NULL_HANDLE;
        REQUIRE(!peer.forced.load());
        REQUIRE(successes(NativeAdrenoMethodBoFree)==beforeFree);
        REQUIRE(successes(NativeAdrenoMethodBoCpuUnmap)==beforeUnmap);
    }
    peer.finish();
    if(!pressure&&!busy&&!fatal){
        OK(vkDeviceWaitIdle(device));
        /* A completed BO is released, or kept whole (handle, mapping, CPU lease) in the BO cache that hands it to the next allocation of
         * its class. */
        const unsigned allocs=successes(NativeAdrenoMethodBoAlloc);
        VkDeviceMemory reuse;OK(vkAllocateMemory(device,&ai,nullptr,&reuse));
        const bool cached=successes(NativeAdrenoMethodBoAlloc)==allocs;
        REQUIRE(cached || (successes(NativeAdrenoMethodBoFree)==beforeFree+1 && successes(NativeAdrenoMethodBoCpuUnmap)==beforeUnmap+1));
        vkFreeMemory(device,reuse,nullptr);
        OK(vkDeviceWaitIdle(device));
    }
    if(busy)inject(NativeAdrenoMethodBoFree,2,1); // temporary busy must retain the IOVA
    if(freeError)inject(NativeAdrenoMethodBoFree,7,1);
    if(leaseError)inject(NativeAdrenoMethodBoCpuUnmap,7,1);
    if(viewError)setenv("HOSTSIM_FAIL_USER_UNMAP","1",1);
    VkResult result=vkAllocateMemory(device,&ai,nullptr,&after);
    if(fatal){
        REQUIRE(result==VK_ERROR_DEVICE_LOST);
        REQUIRE(successes(NativeAdrenoMethodBoFree)==beforeFree);
        REQUIRE(successes(NativeAdrenoMethodBoCpuUnmap)==beforeUnmap+(freeError?1u:0u));
    } else {
        REQUIRE(result==VK_SUCCESS);
        if(busy){
            REQUIRE(lastIova()!=oldIova);REQUIRE(successes(NativeAdrenoMethodBoFree)==beforeFree);
            VkDeviceMemory retry;OK(vkAllocateMemory(device,&ai,nullptr,&retry));
            REQUIRE(successes(NativeAdrenoMethodBoFree)==beforeFree+1);
            REQUIRE(successes(NativeAdrenoMethodBoCpuUnmap)==beforeUnmap+1);
            vkFreeMemory(device,retry,nullptr);
        } else REQUIRE(successes(NativeAdrenoMethodBoFree)>=beforeFree);   /* completed BOs may go to the cache */
    }
    unsetenv("HOSTSIM_FAIL_USER_UNMAP");inject(~0u,0,0);
    if(!fatal)OK(vkDeviceWaitIdle(device));
    if(probe)vkFreeMemory(device,probe,nullptr);
    if(after)vkFreeMemory(device,after,nullptr);
    vkDestroyCommandPool(device,pool,nullptr);vkDestroyBuffer(device,buffer,nullptr);vkFreeMemory(device,used,nullptr);
    vkDestroyDevice(device,nullptr);vkDestroyInstance(instance,nullptr);
    // A failed fence/view/lease release retains native ownership in this narrow peer;
    // it deliberately does not manufacture a successful IOKit close or recovery.
    if(!fatal)REQUIRE(live()==0);
    std::printf("PASS: deferred BO %s; native objects at exit=%u; software control only, no shader execution\n",argv[2],live());
    return 0;
}
