#define _POSIX_C_SOURCE 200809L
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "posix_store.h"
#include "event_display.h"
int main(void)
{
    char directory[]="/tmp/lab5-storage-XXXXXX";assert(mkdtemp(directory));
    char path[256];snprintf(path,sizeof(path),"%s/raft.log",directory);
    struct l5_posix_store disk;struct l5_storage s;assert(!l5_posix_open(&disk,path,&s));
    struct l5_meta meta={.term=1,.voted_for=1,.client_seq=128};assert(!s.save(s.context,&meta));
    struct l5_entry entry={1,1,l5_event(L5_EVENT,2,0,2,1)};assert(!s.append(s.context,&entry));
    meta.commit=meta.applied=1;assert(!s.save(s.context,&meta));
    struct l5_event_display view;assert(!l5_display_open(&view,path));
    assert(!l5_display_repair(&view,&s,1)); /* Repairs a crash between watermark and display. */
    assert(!l5_display_record(&view,&entry,false)); /* Does not duplicate or relabel it live. */
    char event_path[512];snprintf(event_path,sizeof(event_path),"%s.events/00000001.json",path);
    FILE *f=fopen(event_path,"r");assert(f);char line[256];assert(fgets(line,sizeof(line),f));fclose(f);
    assert(strstr(line,"\"replayed\":true"));
    l5_posix_close(&disk);assert(!l5_posix_open(&disk,path,&s));
    uint32_t last;assert(!s.load(s.context,&meta,&last));assert(last==1&&meta.commit==1&&meta.client_seq==128);
    l5_posix_close(&disk);assert(!unlink(event_path));snprintf(event_path,sizeof(event_path),"%s.events",path);assert(!rmdir(event_path));
    assert(!unlink(path));assert(!rmdir(directory));
    puts("PASS POSIX: durable log reload and index-keyed replay display repair/deduplication");
}
