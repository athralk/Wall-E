#ifndef NODE_MEMORY_H
#define NODE_MEMORY_H

/*
 * Remembers the nodes on the way in, so a dead end can be undone.
 *
 * Every new node is pushed with its exits and the exit taken (straight first,
 * like normal). If that exit is a dead end, the bot U-turns and arrives back at
 * the node from the exit it took. node_memory_return() then picks the next
 * exit that hasn't been tried (same order: straight, TURN_PREFERENCE, other)
 * and works out which way that is from the bot's new direction.
 *
 * If every exit of the node was tried, the node is dropped and the bot heads
 * back the way it originally came, still "returning", so the node before it
 * gets the same treatment.
 *
 * Exits are stored as seen when the bot first arrived (original direction).
 */

#include <stdbool.h>
#include "junction.h"

/*
 * Forget everything (start of the run)
 */
void node_memory_reset(void);

/*
 * First visit to a node: remember its exits and the exit we are taking
 */
void node_memory_push(bool left, bool straight, bool right, exit_t taken);

/*
 * Back at the last remembered node after a U-turn. left / straight / right are the
 * exits seen now. Returns the exit to take from the current direction, or EXIT_NONE
 * if nothing is remembered. *still_returning is set when every exit was tried and
 * the bot is heading back toward the node before.
 */
exit_t node_memory_return(bool left, bool straight, bool right, bool *still_returning);

#endif
