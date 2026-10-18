#ifndef BACKTRACK_H
#define BACKTRACK_H

/*
 * Backtracking: "if confused, go back to the last spot where we weren't".
 *
 * Every loop the wheel duties are recorded. Whenever FOLLOW has the line
 * centred under the bar the history is wiped, so it only ever holds the path
 * since the last good spot (up to BACKTRACK_MAX_MS of it).
 *
 * When something goes wrong, start_lost() (declared in maze.h) enters
 * BACKTRACK, which plays the history backwards with the wheels reversed until
 * the centre sensor finds the line again, then goes back to FOLLOW. If the
 * history runs out, or it already backtracked BACKTRACK_TRIES times in a row,
 * it falls back to SEARCH.
 */

/*
 * Call once per loop to save the current wheel duties
 */
void backtrack_record(void);

/*
 * Line is centred: this is a good spot, forget the path before it
 */
void backtrack_mark_good(void);

#endif
