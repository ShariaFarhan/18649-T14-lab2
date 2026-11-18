#ifndef L5_NETWORK_H
#define L5_NETWORK_H
#include <stdbool.h>
#include <stdint.h>
enum l5_event_kind { L5_NOOP, L5_COCKPIT_OK, L5_COCKPIT_ERROR,
    L5_NODE_UP,L5_NODE_DOWN,L5_LINK_UP,L5_LINK_DOWN,L5_EVENT };
/* Bits 9..31 store the request number. Old 16-bit records decode unchanged. */
#define L5_REQUEST_MAX (UINT32_MAX >> 9)
static inline uint32_t l5_event(unsigned kind,unsigned node,unsigned bus,unsigned client,unsigned request)
{ return kind | (node<<4) | (bus<<6) | (client<<7) | (request<<9); }
static inline unsigned l5_kind(uint32_t e) { return e&15; }
static inline unsigned l5_node(uint32_t e) { return (e>>4)&3; }
static inline unsigned l5_bus(uint32_t e) { return (e>>6)&1; }
static inline unsigned l5_client(uint32_t e) { return (e>>7)&3; }
static inline unsigned l5_request(uint32_t e) { return e>>9; }
struct l5_network { uint8_t nodes,links[4];bool cockpit_ok; };
bool l5_event_valid(uint32_t event);
void l5_addLink(struct l5_network *,unsigned node,unsigned bus);
void l5_dropLink(struct l5_network *,unsigned node,unsigned bus);
void l5_addNode(struct l5_network *,unsigned node);
void l5_dropNode(struct l5_network *,unsigned node);
bool l5_event_changes(const struct l5_network *,uint32_t);
void l5_apply(struct l5_network *,uint32_t);
/* Links on this node that reach destination directly or through a bridge.
 * A zero result means no committed route, not permission to invent a link. */
unsigned l5_route(const struct l5_network *,unsigned self,unsigned destination);
#endif
