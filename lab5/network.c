#include "network.h"
#include <string.h>
bool l5_event_valid(uint32_t e)
{
    unsigned k=l5_kind(e),n=l5_node(e);
    if ((e>>25) || k>L5_EVENT) return false;
    if (k==L5_NOOP) return e==0;
    if((k==L5_COCKPIT_OK||k==L5_COCKPIT_ERROR)&&(n!=1||l5_bus(e)!=0))return false;
    if((k==L5_NODE_UP||k==L5_NODE_DOWN)&&l5_bus(e)!=0)return false;
    return n>=1 && n<=3 && l5_client(e)>=1 && l5_request(e)>0;
}
void l5_addLink(struct l5_network*g,unsigned n,unsigned b)
{if(n>=1&&n<=3&&b<2&&(g->nodes&(1u<<n)))g->links[n]|=1u<<b;}
void l5_dropLink(struct l5_network*g,unsigned n,unsigned b)
{if(n>=1&&n<=3&&b<2)g->links[n]&=~(1u<<b);}
void l5_addNode(struct l5_network*g,unsigned n)
{if(n>=1&&n<=3&&!(g->nodes&(1u<<n))){g->nodes|=1u<<n;g->links[n]=0;}}
void l5_dropNode(struct l5_network*g,unsigned n)
{if(n>=1&&n<=3){g->nodes&=~(1u<<n);g->links[n]=0;}}
void l5_apply(struct l5_network *g,uint32_t e)
{
    unsigned n=l5_node(e),b=l5_bus(e);
    switch (l5_kind(e)) {
    case L5_NODE_UP:l5_addNode(g,n);break;
    case L5_NODE_DOWN:l5_dropNode(g,n);break;
    case L5_LINK_UP:l5_addLink(g,n,b);break;
    case L5_LINK_DOWN:l5_dropLink(g,n,b);break;
    case L5_COCKPIT_OK:g->cockpit_ok=true;break;
    case L5_COCKPIT_ERROR:g->cockpit_ok=false;break;
    default:break;
    }
}
bool l5_event_changes(const struct l5_network *g,uint32_t e)
{
    if(l5_kind(e)==L5_EVENT || l5_kind(e)==L5_NOOP)return true;
    struct l5_network next=*g;l5_apply(&next,e);
    return next.nodes!=g->nodes || next.cockpit_ok!=g->cockpit_ok || memcmp(next.links,g->links,4);
}
unsigned l5_route(const struct l5_network *g,unsigned self,unsigned dest)
{
    if(self<1||self>3||dest<1||dest>3||!(g->nodes&(1u<<self))||!(g->nodes&(1u<<dest)))return 0;
    unsigned direct=g->links[self]&g->links[dest];
    if(direct)return direct;
    for(unsigned bridge=1;bridge<=3;bridge++)
        if(bridge!=self && bridge!=dest && (g->nodes&(1u<<bridge)) && g->links[bridge]==3 && g->links[dest])
            return g->links[self];
    return 0;
}
