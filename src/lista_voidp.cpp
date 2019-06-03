// author: Mihai Oltean
// email: mihai.oltean@gmail.com
// main website: https://www.jenny5.org
// mirror website: https://jenny5-robot.github.io
// source code: https://github.com/jenny5-robot

// MIT License
// ---------------------------------------------------------------------------

#include "lista_voidp.h"
#include <stdio.h>

// ---------------------------------------------------------------------------
t_lista::t_lista()
{
	head = NULL;
	tail = NULL;
	count = 0;
}

// ---------------------------------------------------------------------------
t_lista::t_lista(t_lista& c)
{
	head = NULL;
	tail = NULL;

	t_node_double_linked *tmp = c.head;
	while (tmp)
	{
		Add(tmp->inf);
		tmp = tmp->next;
	}
	count = c.count;
}

// ---------------------------------------------------------------------------
t_lista::~t_lista()
{
	while (head)
	{
		t_node_double_linked * tmp = head;
		head = head->next;
		delete tmp;
	}
	tail = NULL;
	count = 0;
}

// ---------------------------------------------------------------------------
void t_lista::Add(void* data)
{
	if (!head)
	{
		head = new t_node_double_linked;
		head->inf = data;
		head->next = NULL;
		head->prev = NULL;
		tail = head;
	}
	else
	{
		t_node_double_linked* tmp = new t_node_double_linked;
		tmp->inf = data;
		tmp->next = NULL;
		tmp->prev = tail;
		tail->next = tmp;
		tail = tmp;
	}
	count++;
}

// ---------------------------------------------------------------------------
void t_lista::Delete(int Index)
{
	if (Index < count)
		if (head)
		{
			if (!Index)
			{ // delete the head
				t_node_double_linked* tmp = head;
				head = head->next;
				if (head)
					head->prev = NULL;
				delete tmp;
				if (count == 1)
					tail = NULL;
			}
			else
			{ // delete the other
				int i = 0;
				t_node_double_linked* tmp = head;
				while (tmp && (i < Index - 1))
				{
					i++;
					tmp = tmp->next;
				}
				// sunt pozitionat pe anterioru
				if (tmp)
				{
					t_node_double_linked* urm = tmp->next;
					tmp->next = urm->next;
					if (urm->next)
						urm->next->prev = tmp;
					delete urm;
					if (Index == count - 1)
						tail = tmp;
				}
			}
			count--;
		}
}

// ---------------------------------------------------------------------------
/*
 void t_lista::Insert(int Index, void* data)
 {

 if (head == NULL)
 Add(data);
 else{
 int i = 0;
 t_node_double_linked* tmp = head;
 while ((tmp != NULL) && (i < Index)){
 i++;
 tmp = tmp->next;
 }
 }

 }
 */
// ---------------------------------------------------------------------------
void* t_lista::GetInfo(int Index)
{
	if (!head)
		return NULL;

	int i = 0;
	t_node_double_linked* tmp = head;
	while (tmp && (i < Index))
	{
		i++;
		tmp = tmp->next;
	}
	if (tmp)
		return tmp->inf;
	else
		return NULL;
}

// ---------------------------------------------------------------------------
t_node_double_linked* t_lista::GetNode(int Index)
{
	if (!head)
		return NULL;

	int i = 0;
	t_node_double_linked* tmp = head;
	while (tmp && (i < Index))
	{
		i++;
		tmp = tmp->next;
	}
	if (tmp)
		return tmp;
	else
		return NULL;
}

// ---------------------------------------------------------------------------
/*
 t_lista& t_lista::operator=(const t_lista& c)
 {
 if (this == &c)
 return *this;
 else{
 if (this->count > 0) // trebuie sa o sterg prima data
 while (this->count > 0)
 this->Delete(0);
 t_node_double_linked *tmp = c.head;
 while (tmp != NULL){
 this->Add(tmp->inf);
 tmp = tmp->next;
 }
 return *this;
 }
 }
 */
// ---------------------------------------------------------------------------
/*
 void* t_lista::GetCurrentInfo(t_node_double_linked* p)
 {
 if (p)
 return p->inf;
 return NULL;
 }
 */
// ---------------------------------------------------------------------------
void* t_lista::GetNextInfo(t_node_double_linked* p)
{
	if (p){
		if (p->next)
			return p->next->inf;
		else
			return NULL;
    }
	return NULL;
}

// ---------------------------------------------------------------------------
void* t_lista::GetPrevInfo(t_node_double_linked* p)
{
	if (p){
		if (p->prev)
			return p->prev->inf;
		else
			return NULL;
    }
	return NULL;
}

// ---------------------------------------------------------------------------
void* t_lista::GetNextCircularInfo(t_node_double_linked* p)
{
	if (p){
		if (!p->next)
			return head->inf;
		else
			return p->next->inf;
    }
	return NULL;
}

// ---------------------------------------------------------------------------
void* t_lista::GetPrevCircularInfo(t_node_double_linked* p)
{
	if (p){
		if (!p->prev)
			return tail->inf;
		else
			return p->prev->inf;
    }
	return NULL;
}

// ---------------------------------------------------------------------------
t_node_double_linked* t_lista::delete_current_circular(t_node_double_linked* p)
{
	if (count > 1) {
		// more elements
		if (p == head)
			head = head->next;
		if (p == tail)
			tail = tail->prev;

		p->prev->next = p->next;
		p->next->prev = p->prev;
		delete p;
		count--;
		return head;
	}
	else {
		// one element
		delete p;
		tail = NULL;
		head = NULL;
		count = 0;
		return NULL;
	}
}
// ---------------------------------------------------------------------------
t_node_double_linked* t_lista::DeleteCurrent(t_node_double_linked* p)
{
	if (count > 0)
		if (p == head)
		{
			head = head->next;
			if (head)
				head->prev = NULL;
			delete p;
			// p = NULL;
			if (count == 1)
				tail = NULL;
			count--;
			return head;
		}
		else
			if (p == tail)
			{
				tail = tail->prev;
				if (tail)
					tail->next = NULL;
				delete p;
				// p = NULL;
				if (count == 1)
					head = NULL;
				count--;
				return NULL;
			}
			else
			{
				t_node_double_linked *tmp = p->next;
				p->prev->next = p->next;
				p->next->prev = p->prev;
				delete p;
				count--;
				return tmp;
			}
	else
		return NULL;
}

// ---------------------------------------------------------------------------
void t_lista::Append(t_lista & )
{

}

// ---------------------------------------------------------------------------
void t_lista::Insert(long Index, void* data)
{

	if (Index >= count)
		Add(data);
	else
		if (Index == 0)
		{ // insert in the head
			t_node_double_linked* k = new t_node_double_linked;
			k->inf = data;
			k->prev = NULL;
			k->next = head;
			head->prev = k;
			head = k;
			count++;
		}
		else
		{
			long i = 0;
			t_node_double_linked* tmp = head;
			while (tmp && (i < Index - 1))
			{
				i++;
				tmp = tmp->next;
			}
			// insert AFTER tmp
			if (tmp)
			{
				t_node_double_linked* k = new t_node_double_linked;
				k->inf = data;
				k->prev = tmp;
				k->next = tmp->next;
				tmp->next->prev = k;
				tmp->next = k;
				count++;
			}
		}
}

// ---------------------------------------------------------------------------
void t_lista::Clear(void)
{
	while (head)
	{
		t_node_double_linked * tmp = head;
		head = head->next;
		delete tmp;
	}
	tail = NULL;
	head = NULL;
	count = 0;
}

// ---------------------------------------------------------------------------
void t_lista::DeleteHead(void)
{
	if (head)
	{
		t_node_double_linked* tmp = head;
		head = head->next;
		if (head)
			head->prev = NULL;
		delete tmp;
		if (count == 1)
			tail = NULL;
		count--;
	}
}

// ---------------------------------------------------------------------------
	void* t_lista::GetCurrentInfo(t_node_double_linked* p){
		if (p)
			return p->inf;
		return 0;
	}
// ---------------------------------------------------------------------------
	void* t_lista::GetHeadInfo(void){
		if (head)
			return head->inf;
		return 0;
	}
// ---------------------------------------------------------------------------
void AppendWithoutCopy(t_lista &)
{

}
// ---------------------------------------------------------------------------
void t_lista::make_circular(void)
{
	if (head) {
		head->prev = tail;
		tail->next = head;
	}
}
// ---------------------------------------------------------------------------
