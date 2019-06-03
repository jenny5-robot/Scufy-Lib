// author: Mihai Oltean, 
// email: mihai.oltean@gmail.com
// main website: https://www.jenny5.org
// mirror website: https://jenny5-robot.github.io
// source code: https://github.com/jenny5-robot

// MIT License
// ---------------------------------------------------------------------------

#ifndef lista_H
#define lista_H
//---------------------------------------------------------------------------

struct t_node_double_linked{
  void* inf;
  t_node_double_linked* next, *prev;
};
//---------------------------------------------------------------------------
class t_lista{
	private:
	public:
		t_node_double_linked* head, *tail;
		int count;
		t_lista();
		t_lista(t_lista&);
		~t_lista();
		void Add(void*);
		void Delete(int);
		t_node_double_linked* DeleteCurrent(t_node_double_linked*);
		void* GetInfo(int);
		t_node_double_linked* GetNode(int Index);
		void Append(t_lista& source);
		void Insert(long, void*);
		void Clear(void);
		void DeleteHead(void);

		void* GetCurrentInfo(t_node_double_linked* p);
		void* GetHeadInfo(void);

		void* GetNextInfo(t_node_double_linked*);
		void* GetPrevInfo(t_node_double_linked*);
		void* GetNextCircularInfo(t_node_double_linked*);
		void* GetPrevCircularInfo(t_node_double_linked*);
		t_node_double_linked* delete_current_circular(t_node_double_linked* p);

		void AppendWithoutCopy(t_lista&source);
		void make_circular(void);

};
//---------------------------------------------------------------------------
#endif
